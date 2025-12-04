#include "psaf_firststeps/lane_detection_node.hpp"

#include <algorithm>
#include <ctime>
#include <functional>
#include <string>
#include <vector>

LaneDetectionNode::LaneDetectionNode()
: Node{"lane_detection"}
{
  save_debug_images_ = this->declare_parameter("save_debug_images", false);

  const auto qos = rclcpp::QoS{rclcpp::KeepLast(1)};
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "color/image_raw", qos,
    std::bind(&LaneDetectionNode::imageCallback, this, std::placeholders::_1));

  state_sub_ = this->create_subscription<psaf_firststeps::msg::State>(
    "state_machine/state", qos,
    std::bind(&LaneDetectionNode::stateCallback, this, std::placeholders::_1));

  lane_pub_ = this->create_publisher<psaf_firststeps::msg::LaneMarkings>(
    "lane_detection/lane_markings", qos);

  last_lane_.center_valid = false;
}

double LaneDetectionNode::computeOrientation(const cv::Mat & points)
{
  if (points.empty()) {
    return 0.0;
  }

  cv::Mat points32f;
  if (points.type() == CV_32F) {
    points32f = points;
  } else {
    points.convertTo(points32f, CV_32F);
  }

  cv::Mat mean;
  cv::reduce(points32f, mean, 0, cv::REDUCE_AVG);
  cv::Mat data = points32f - cv::repeat(mean, points32f.rows, 1);
  cv::Mat cov = (data.t() * data) / std::max(points32f.rows - 1, 1);
  cv::Mat eigenvalues;
  cv::Mat eigenvectors;
  cv::eigen(cov, eigenvalues, eigenvectors);
  cv::Vec2d main_axis(eigenvectors.row(0));
  double angle_deg = std::atan2(main_axis[1], main_axis[0]) * 180.0 / CV_PI;
  if (angle_deg < 0) {
    angle_deg += 180.0;
  }
  return angle_deg;
}

cv::Mat LaneDetectionNode::extract10EdgePoints(const cv::Mat & points)
{
  if (points.empty()) {
    return cv::Mat();
  }

  std::vector<cv::Point> pts;
  pts.reserve(points.rows);
  for (int i = 0; i < points.rows; ++i) {
    pts.emplace_back(
      static_cast<int>(points.at<float>(i, 0)),
      static_cast<int>(points.at<float>(i, 1))
    );
  }

  if (pts.size() < 2) {
    return cv::Mat();
  }

  std::vector<cv::Point> hull;
  cv::convexHull(pts, hull);

  if (hull.size() < 2) {
    return cv::Mat();
  }

  std::sort(hull.begin(), hull.end(), [](const cv::Point & a, const cv::Point & b) {
    return a.y < b.y;
  });

  cv::Mat sampled(10, 2, CV_32F);
  for (int i = 0; i < 10; ++i) {
    int idx = static_cast<int>(i * (hull.size() - 1) / 9.0);
    sampled.at<float>(i, 0) = static_cast<float>(hull[idx].x);
    sampled.at<float>(i, 1) = static_cast<float>(hull[idx].y);
  }

  return sampled;
}

cv::Mat LaneDetectionNode::getLaneContour(const cv::Mat & labels, int component_id)
{
  cv::Mat mask = labels == component_id;
  mask.convertTo(mask, CV_8U, 255);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  if (contours.empty() || contours[0].size() < 2) {
    return cv::Mat();
  }
  cv::Mat contour = cv::Mat(contours[0]).reshape(1, contours[0].size());
  contour.convertTo(contour, CV_32F);
  return contour;
}

std::vector<geometry_msgs::msg::Point> LaneDetectionNode::detectLane(const cv::Mat & image)
{
  std::vector<geometry_msgs::msg::Point> right_lane;

  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  cv::Mat binary;
  cv::adaptiveThreshold(
    gray, gray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 51, -20);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(gray, binary, cv::MORPH_OPEN, kernel);

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids);

  cv::Mat lane_mask = cv::Mat::zeros(binary.size(), CV_8U);
  const int min_component_size = 700;
  constexpr double angle_min = 12.0;
  constexpr double angle_max = 130.0;

  for (int i = 1; i < num_labels; ++i) {
    cv::Mat points_raw;
    cv::findNonZero(labels == i, points_raw);
    if (points_raw.empty() || points_raw.rows < min_component_size) {
      continue;
    }

    cv::Mat points(points_raw.rows, 2, CV_32F);
    for (int j = 0; j < points_raw.rows; ++j) {
      const cv::Point p = points_raw.at<cv::Point>(j);
      points.at<float>(j, 0) = static_cast<float>(p.x);
      points.at<float>(j, 1) = static_cast<float>(p.y);
    }

    const double angle = computeOrientation(points);
    if (angle >= angle_min && angle <= angle_max) {
      lane_mask.setTo(255, labels == i);
    }
  }

  cv::Mat labels_lane;
  cv::Mat stats_lane;
  cv::Mat centroids_lane;
  const int num_labels_lane = cv::connectedComponentsWithStats(
    lane_mask, labels_lane, stats_lane, centroids_lane);

  struct Lane
  {
    int label;
    cv::Mat pixels;
    cv::Point2d centroid;
  };

  std::vector<Lane> lane_structures;
  for (int i = 1; i < num_labels_lane; ++i) {
    const int area = stats_lane.at<int>(i, cv::CC_STAT_AREA);
    if (area < min_component_size) {
      continue;
    }

    cv::Mat points_raw;
    cv::findNonZero(labels_lane == i, points_raw);
    if (points_raw.empty()) {
      continue;
    }

    cv::Mat points(points_raw.rows, 2, CV_32F);
    for (int j = 0; j < points_raw.rows; ++j) {
      const cv::Point p = points_raw.at<cv::Point>(j);
      points.at<float>(j, 0) = static_cast<float>(p.x);
      points.at<float>(j, 1) = static_cast<float>(p.y);
    }

    const cv::Point2d centroid(centroids_lane.at<double>(i, 0), centroids_lane.at<double>(i, 1));
    lane_structures.push_back({i, points, centroid});
  }

  std::vector<Lane> lane_bottom;
  const int height = lane_mask.rows;
  for (auto & lane : lane_structures) {
    cv::Mat col1 = lane.pixels.col(1);
    if (cv::countNonZero(col1 >= height - 100) > 0) {
      lane_bottom.push_back(lane);
    }
  }

  std::sort(
    lane_bottom.begin(), lane_bottom.end(),
    [](const Lane & a, const Lane & b) {return a.centroid.x > b.centroid.x;});

  std::vector<Lane> detected_lanes;
  for (size_t i = 0; i < std::min<size_t>(lane_bottom.size(), 3); ++i) {
    detected_lanes.push_back(lane_bottom[i]);
  }

  cv::Mat pts10;
  if (!detected_lanes.empty()) {
    cv::Mat contour = getLaneContour(labels_lane, detected_lanes.front().label);
    if (!contour.empty() && contour.cols == 2) {
      pts10 = extract10EdgePoints(contour);
    }
  }

  if (save_debug_images_) {
    cv::Mat vis;
    image.copyTo(vis);
    for (const auto & lane : lane_bottom) {
      for (int i = 0; i < lane.pixels.rows; ++i) {
        const int x = static_cast<int>(lane.pixels.at<float>(i, 0));
        const int y = static_cast<int>(lane.pixels.at<float>(i, 1));
        cv::circle(vis, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
      }
    }
    const std::string filename =
      "detected_lanes_" + std::to_string(std::time(nullptr)) + ".png";
    cv::imwrite(filename, vis);
  }

  if (pts10.empty()) {
    return right_lane;
  }

  const cv::Mat H = (cv::Mat_<double>(3, 3) <<
      -5.428312102536678, -8.791254779610195, 3621.340810770122,
      -0.22533640658223836, -41.47045031646772, 5045.52787664645,
      -0.0001767182243341023, -0.03633888349942347, 1.0);

  constexpr double s_E = 0.4795;
  constexpr double n_0 = 1199.6637;
  constexpr double s_0 = 239.32;

  for (int i = 0; i < pts10.rows; ++i) {
    cv::Mat pt = (cv::Mat_<double>(3, 1) << pts10.at<float>(i, 0), pts10.at<float>(i, 1), 1.0);
    cv::Mat transformed = H * pt;
    const double x_pix = transformed.at<double>(0, 0) / transformed.at<double>(2, 0);
    const double y_pix = transformed.at<double>(1, 0) / transformed.at<double>(2, 0);

    geometry_msgs::msg::Point p;
    p.x = (-y_pix + n_0) / s_E;
    p.y = (-x_pix + s_0) / s_E;
    p.z = 0.0;
    right_lane.push_back(p);
  }

  return right_lane;
}

void LaneDetectionNode::stateCallback(const psaf_firststeps::msg::State::SharedPtr msg)
{
  active_ = msg->state == psaf_firststeps::msg::State::DRIVING;
}

void LaneDetectionNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!active_) {
    return;
  }

  try {
    const auto image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    const auto right_lane = detectLane(image_ptr->image);
    publishLane(msg->header.stamp, right_lane);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
  }
}

void LaneDetectionNode::publishLane(
  const rclcpp::Time & stamp, const std::vector<geometry_msgs::msg::Point> & right_lane)
{
  psaf_firststeps::msg::LaneMarkings lane_msg;
  lane_msg.right_lane = right_lane;
  lane_msg.center_valid = false;
  lane_msg.stamp = stamp;
  last_lane_ = lane_msg;
  lane_pub_->publish(lane_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
