if (!require("data.table")){
    install.packages("data.table", repos="http://cran.rstudio.com/")
    library("data.table")
}

## ---- read_data
file_name <- paste0("turtlebot", params$robot, "_gazebo_odometry_filtered.csv")
t1_gazebo <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_continuous_odometry_filtered.csv")
t1_continuous <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_discrete_odometry_filtered.csv")
t1_discrete <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_external_pose_count.csv")
t1_external_count <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

## ---- calculations
t1_gazebo$dist_from_origin <- sqrt(t1_gazebo$x_position ^ 2 + t1_gazebo$y_position ^ 2)

t1_discrete$x_error <- t1_gazebo$x_position - t1_discrete$x_position
t1_discrete$y_error <- t1_gazebo$y_position - t1_discrete$y_position
t1_discrete$dist_error <- sqrt(t1_discrete$x_error ^ 2 + t1_discrete$y_error ^ 2)

t1_continuous$x_error <- t1_gazebo$x_position - t1_continuous$x_position
t1_continuous$y_error <- t1_gazebo$y_position - t1_continuous$y_position
t1_continuous$dist_error <- sqrt(t1_continuous$x_error ^ 2 + t1_continuous$y_error ^ 2)

## ---- plot
plot(t1_gazebo$x_position, t1_gazebo$y_position)
title("Ground truth visited locations of robot")

plot(t1_gazebo$dist_from_origin)
title("Distance from origin vs. time")

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

## ---- summary
summary(t1_continuous$x_error)
summary(t1_continuous$y_error)
summary(t1_continuous$dist_error)

summary(t1_discrete$x_error)
summary(t1_discrete$y_error)
summary(t1_discrete$dist_error)

## ---- time
total_time_seconds <- (length(t1_gazebo$x_position) / 10)
total_time_minutes <- total_time_seconds / 60
total_time_hours <- total_time_minutes / 60
partial_time_hours <- floor(total_time_hours)
partial_time_minutes <- floor((total_time_hours - partial_time_hours) * 60)
partial_time_seconds <- (total_time_minutes - partial_time_minutes) * 60

## ---- external_poses
external_poses <- t1_external_count$count[length(t1_external_count$count)]
