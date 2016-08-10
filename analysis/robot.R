## ---- dependencies
if (!require(data.table)){
    install.packages("data.table", repos="http://cran.rstudio.com/")
    library(data.table)
}

if (!require("fitdistrplus")){
    install.packages("fitdistrplus", repos="https://cran.rstudio.com/")
    library("fitdistrplus")
}

## ---- read_data
file_name <- paste0("turtlebot", params$robot, "_gazebo_odom.csv")
gazebo <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_continuous_filter_odom.csv")
continuous <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_discrete_filter_odom.csv")
discrete <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <- paste0("turtlebot", params$robot, "_external_pose_count.csv")
external_count <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <-paste0("turtlebot", params$robot, "_gps_data.csv")
gps <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

file_name <-paste0("turtlebot", params$robot, "_noisy_odom_data.csv")
noisy_odom <- fread(paste(params$data_dir, params$experiment, file_name, sep="/"), header=T, sep=",")

## ---- calculations
gazebo$dist_from_origin <- sqrt(gazebo$x_position ^ 2 + gazebo$y_position ^ 2)

discrete$x_error <- gazebo$x_position - discrete$x_position
discrete$y_error <- gazebo$y_position - discrete$y_position
discrete$dist_error <- sqrt(discrete$x_error ^ 2 + discrete$y_error ^ 2)

continuous$x_error <- gazebo$x_position - continuous$x_position
continuous$y_error <- gazebo$y_position - continuous$y_position
continuous$dist_error <- sqrt(continuous$x_error ^ 2 + continuous$y_error ^ 2)

correct_yaw <- function(yaw) {
    while (yaw > pi) {
        yaw <- yaw - 2*pi
    }
    while (yaw < -1*pi) {
        yaw <- yaw + 2*pi
    }

    return(yaw)
}

vector_correct_yaw <- Vectorize(correct_yaw)
discrete$yaw_error <- vector_correct_yaw(gazebo$yaw - discrete$yaw)
continuous$yaw_error <- vector_correct_yaw(gazebo$yaw - continuous$yaw)

noisy_odom$x_err <- gazebo$x_position - noisy_odom$x
noisy_odom$y_err <- gazebo$y_position - noisy_odom$y
noisy_odom$dist_err <- sqrt(noisy_odom$x_err ^ 2 + noisy_odom$y_err ^ 2)

gps$x_err <- 0 - gps$x
gps$y_err <- 0 - gps$y
gps$dist_err <- sqrt(gps$x_err ^ 2 + gps$y_err ^ 2)

## ---- plot
plot(gazebo$x_position,
     main = "X coordinate of robot over time")

plot(gazebo$y_position,
     main = "Y coordinate of robot over time")

plot(gazebo$dist_from_origin,
     main="Distance from origin vs. time")

plot(continuous$x_error,
     main="Continuous x_error over time")

plot(discrete$x_error,
     main="Discrete x_error over time")

plot(continuous$y_error,
     main="Continuous y_error over time")

plot(discrete$y_error,
     main="Discrete y_error over time")

plot(continuous$yaw_error,
     main="Continuous yaw error over time")

plot(discrete$yaw_error,
     main="Discrete yaw error over time")

plot(continuous$dist_error,
     main="Continuous total distance error over time")

plot(discrete$dist_error,
      main="Discrete total distance error over time")

if (NROW(gps) > 0) {
    plot(gps$x_err,
         main="GPS X Error Over Time")

    plot(gps$y_err,
         main="GPS Y Error Over Time")

    plot(gps$dist_err,
         main="GPS Horizontal Distance Error Over Time")
}

plot(continuous$x_variance,
     main="Continuous Filter X Variance Over Time")

plot(continuous$y_variance,
     main="Continuous Filter Y Variance Over Time")

plot(continuous$yaw_variance,
     main="Continuous Filter Yaw Variance Over Time")

plot(discrete$x_variance,
     main="Discrete Filter X Variance Over Time")

plot(discrete$y_variance,
     main="Discrete Filter Y Variance Over Time")

plot(discrete$yaw_variance,
     main="Discrete Filter Yaw Variance Over Time")

if (NROW(noisy_odom) > 0) {
    plot(noisy_odom$x_err,
         main="Noisy Odom X Error Over Time")

    plot(noisy_odom$y_err,
         main="Noisy Odom Y Error Over Time")

    plot(noisy_odom$dist_err,
         main="Noisy Odom Horizontal Distance Error Over Time")

    plot(noisy_odom$x_variance,
         main="Variance of X Coordinate in Noisy Odometry")

    plot(noisy_odom$y_variance,
         main="Variance of Y Coordinate in Noisy Odometry")

    plot(noisy_odom$yaw_variance,
         main="Variance of Yaw Coordinate in Noisy Odometry")

    plot(noisy_odom$x_vel, noisy_odom$x_variance,
         main="Variance vs. Velocity of X in Noisy Odometry")

    plot(noisy_odom$yaw_vel, noisy_odom$yaw_variance,
         main="Variance vs. Velocity of Yaw in Noisy Odometry")
}

## ---- summary
summary(continuous$x_error)
summary(continuous$y_error)
summary(continuous$yaw_error)
summary(continuous$dist_error)

summary(discrete$x_error)
summary(discrete$y_error)
summary(discrete$yaw_error)
summary(discrete$dist_error)

summary(noisy_odom$x_err)
summary(noisy_odom$y_err)
summary(noisy_odom$dist_err)

if (NROW(gps) > 0) {
    summary(gps$x_err)
    summary(gps$y_err)
    summary(gps$dist_err)
}

if (NROW(noisy_odom) > 0) {
    summary(noisy_odom$x_variance)
    summary(noisy_odom$y_variance)
    summary(noisy_odom$yaw_variance)
}

## ---- time
total_time_seconds <- (length(gazebo$x_position) / 10)
total_time_minutes <- total_time_seconds / 60
total_time_hours <- total_time_minutes / 60
partial_time_hours <- floor(total_time_hours)
partial_time_minutes <- floor((total_time_hours - partial_time_hours) * 60)
partial_time_seconds <- ((total_time_hours - partial_time_hours) * 60 - partial_time_minutes) * 60

## ---- external_poses
external_poses <- external_count$count[length(external_count$count)]
