t1_gazebo <- read.csv("~/thesis/experiment_data/one_mobile/turtlebot1_gazebo_odometry_filtered.csv")
t1_continuous <- read.csv("~/thesis/experiment_data/one_mobile/turtlebot1_continuous_odometry_filtered.csv")
t1_discrete <- read.csv("~/thesis/experiment_data/one_mobile/turtlebot1_discrete_odometry_filtered.csv")
t1_external_count <- read.csv("~/thesis/experiment_data/one_mobile/turtlebot1_external_pose_count.csv")

t1_gazebo$dist_from_origin <- sqrt(t1_gazebo$x_position ^ 2 + t1_gazebo$y_position ^ 2)

t1_discrete$x_error <- t1_gazebo$x_position - t1_discrete$x_position
t1_discrete$y_error <- t1_gazebo$y_position - t1_discrete$y_position
t1_discrete$dist_error <- sqrt(t1_discrete$x_error ^ 2 + t1_discrete$y_error ^ 2)

t1_continuous$x_error <- t1_gazebo$x_position - t1_continuous$x_position
t1_continuous$y_error <- t1_gazebo$y_position - t1_continuous$y_position
t1_continuous$dist_error <- sqrt(t1_continuous$x_error ^ 2 + t1_continuous$y_error ^ 2)

pdf("one_mobile_ground_truth_locations.pdf")
plot(t1_gazebo$x_position, t1_gazebo$y_position)
title("Ground truth visited locations of robot")
dev.off()

pdf("one_mobile_dist_from_origin.pdf")
plot(t1_gazebo$dist_from_origin)
title("Distance from origin vs. time")
dev.off()

summary(t1_discrete$x_error)
summary(t1_discrete$y_error)
summary(t1_discrete$dist_error)

summary(t1_continuous$x_error)
summary(t1_continuous$y_error)
summary(t1_continuous$dist_error)
