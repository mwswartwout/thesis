if (!require("data.table")){
    install.packages("data.table", repos="http://cran.rstudio.com/")
    library("data.table")
}

## ---- read_data
file_name <- paste0("turtlebot", params$robot, "_gazebo_odometry_filtered.csv")
gazebo <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_continuous_odometry_filtered.csv")
continuous <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_discrete_odometry_filtered.csv")
discrete <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_external_pose_count.csv")
external_count <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

## ---- calculations
gazebo$dist_from_origin <- sqrt(gazebo$x_position ^ 2 + gazebo$y_position ^ 2)

discrete$x_error <- gazebo$x_position - discrete$x_position
discrete$y_error <- gazebo$y_position - discrete$y_position
discrete$dist_error <- sqrt(discrete$x_error ^ 2 + discrete$y_error ^ 2)

continuous$x_error <- gazebo$x_position - continuous$x_position
continuous$y_error <- gazebo$y_position - continuous$y_position
continuous$dist_error <- sqrt(continuous$x_error ^ 2 + continuous$y_error ^ 2)

## ---- plot
plot(gazebo$x_position, gazebo$y_position)
title("Ground truth visited locations of robot")

plot(gazebo$dist_from_origin)
title("Distance from origin vs. time")

plot(continuous$x_error)
title("Continuous x_error over time")

plot(continuous$y_error)
title("Continuous y_error over time")

plot(continuous$dist_error)
title("Continuous total distance error over time")

plot(discrete$x_error)
title("Discrete x_error over time")

plot(discrete$y_error)
title("Discrete y_error over time")

plot (discrete$dist_error)
title("Discrete total distance error over time")

## ---- summary
summary(continuous$x_error)
summary(continuous$y_error)
summary(continuous$dist_error)

summary(discrete$x_error)
summary(discrete$y_error)
summary(discrete$dist_error)

## ---- time
total_time_seconds <- (length(gazebo$x_position) / 10)
total_time_minutes <- total_time_seconds / 60
total_time_hours <- total_time_minutes / 60
partial_time_hours <- floor(total_time_hours)
partial_time_minutes <- floor((total_time_hours - partial_time_hours) * 60)
partial_time_seconds <- (total_time_minutes - partial_time_minutes) * 60

## ---- external_poses
external_poses <- external_count$count[length(external_count$count)]
