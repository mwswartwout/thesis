params <- data.frame(data_dir="", experiment_name="")

params$data_dir <- "/home/matt/thesis/experiment_data"
params$experiment_name <- "one_mobile"

t1_gazebo <- read.csv(paste(params$data_dir, params$experiment_name, "turtlebot1_gazebo_odometry_filtered.csv", sep="/"))
t1_continuous <- read.csv(paste(params$data_dir, params$experiment_name, "turtlebot1_continuous_odometry_filtered.csv", sep="/"))
t1_discrete <- read.csv(paste(params$data_dir, params$experiment_name, "turtlebot1_discrete_odometry_filtered.csv", sep="/"))
t1_external_count <- read.csv(paste(params$data_dir, params$experiment_name, "turtlebot1_external_pose_count.csv", sep="/"))

t1_gazebo$dist_from_origin <- sqrt(t1_gazebo$x_position ^ 2 + t1_gazebo$y_position ^ 2)

t1_discrete$x_error <- t1_gazebo$x_position - t1_discrete$x_position
t1_discrete$y_error <- t1_gazebo$y_position - t1_discrete$y_position
t1_discrete$dist_error <- sqrt(t1_discrete$x_error ^ 2 + t1_discrete$y_error ^ 2)

t1_continuous$x_error <- t1_gazebo$x_position - t1_continuous$x_position
t1_continuous$y_error <- t1_gazebo$y_position - t1_continuous$y_position
t1_continuous$dist_error <- sqrt(t1_continuous$x_error ^ 2 + t1_continuous$y_error ^ 2)

#pdf(paste0(params$experiment_name, "_ground_truth_locations.pdf"))
plot(t1_gazebo$x_position, t1_gazebo$y_position)
title("Ground truth visited locations of robot")
#dev.off()

#pdf(paste0(params$experiment_name, "_dist_from_origin.pdf"))
plot(t1_gazebo$dist_from_origin)
title("Distance from origin vs. time")
#dev.off()

plot(t1_continuous$x_error)
title("Continuous x_error over time")

plot(t1_continuous$y_error)
title("Continuous y_error over time")

plot(t1_continuous$dist_error)
title("Continuous total distance error over time")

plot(t1_discrete$x_error)
title("Discrete x_error over time")

plot(t1_discrete$y_error)
title("Discrete y_error over time")

plot (t1_discrete$dist_error)
title("Discrete total distance error over time")

summary(t1_continuous$x_error)
summary(t1_continuous$y_error)
summary(t1_continuous$dist_error)

summary(t1_discrete$x_error)
summary(t1_discrete$y_error)
summary(t1_discrete$dist_error)
