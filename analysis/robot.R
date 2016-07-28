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
discrete$yaw_error <- gazebo$yaw - discrete$yaw
discrete$dist_error <- sqrt(discrete$x_error ^ 2 + discrete$y_error ^ 2)

continuous$x_error <- gazebo$x_position - continuous$x_position
continuous$y_error <- gazebo$y_position - continuous$y_position
continuous$yaw_error <- gazebo$yaw - continuous$yaw
continuous$dist_error <- sqrt(continuous$x_error ^ 2 + continuous$y_error ^ 2)

noisy_odom$x_err <- gazebo$x_position - noisy_odom$x
noisy_odom$y_err <- gazebo$y_position - noisy_odom$y
noisy_odom$dist_err <- sqrt(noisy_odom$x_err ^ 2 + noisy_odom$y_err ^ 2)

gps$x_err <- gazebo$x_position - gps$x
gps$y_err <- gazebo$y_position - gps$y
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

plot(noisy_odom$x_err,
     main="Noisy Odom X Error Over Time")

plot(noisy_odom$y_err,
     main="Noisy Odom Y Error Over Time")

plot(noisy_odom$dist_err,
     main="Noisy Odom Horizontal Distance Error Over Time")

plot(gps$x_err,
     main="GPS X Error Over Time")

plot(gps$y_err,
     main="GPS Y Error Over Time")

plot(gps$dist_err,
     main="GPS Horizontal Distance Error Over Time")

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

summary(gps$x_err)
summary(gps$y_err)
summary(gps$dist_err)

#sum(continuous$dist_error <= 0.25) / length(continuous$dist_error)
#sum(discrete$dist_error <= 0.25) / length(discrete$dist_error)

shapiro.test(noisy_odom$x_err)
shapiro.test(noisy_odom$y_err)
shapiro.test(noisy_odom$dist_err)

shapiro.test(gps$x_err)
shapiro.test(gps$y_err)
shapiro.test(gps$dist_err)

noisy_odom_fit_x <- fitdist(noisy_odom$x_err, "norm")
noisy_odom_fit_y <- fitdist(noisy_odom$y_err, "norm")
noisy_odom_fit_dist <- fitdist(noisy_odom$dist_err, "norm")

summary(noisy_odom_fit_x)
summary(noisy_odom_fit_y)
summary(noisy_odom_fit_dist)

gps_fit_x <- fitdist(gps$x_err, "norm")
gps_fit_y <- fitdist(gps$y_err, "norm")
gps_fit_dist <- fitdist(gps$dist_err, "norm")

summary(gps_fit_x)
summary(gps_fit_y)
summary(gps_fit_dist)

## ---- time
total_time_seconds <- (length(gazebo$x_position) / 10)
total_time_minutes <- total_time_seconds / 60
total_time_hours <- total_time_minutes / 60
partial_time_hours <- floor(total_time_hours)
partial_time_minutes <- floor((total_time_hours - partial_time_hours) * 60)
partial_time_seconds <- ((total_time_hours - partial_time_hours) * 60 - partial_time_minutes) * 60

## ---- external_poses
external_poses <- external_count$count[length(external_count$count)]
