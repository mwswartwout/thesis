if (!require("data.table")){
    install.packages("data.table", repos="http://cran.rstudio.com/")
    library("data.table")
}

# First we need to get data from each robot and combine into one data set
## ---- read_data
for (robot in 1:params$robots) {
    gazebo_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_gazebo_odometry_filtered.csv")})
    gazebo_full_filenames <- lapply(gazebo_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    gazebo_data <- rbindlist(lapply(gazebo_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    discrete_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_discrete_odometry_filtered.csv")})
    discrete_full_filenames <- lapply(discrete_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    discrete_data <- rbindlist(lapply(discrete_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    continuous_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_continuous_odometry_filtered.csv")})
    continuous_full_filenames <- lapply(continuous_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    continuous_data <- rbindlist(lapply(continuous_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    external_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_external_pose_count.csv")})
    external_full_filenames <- lapply(external_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    external_data_tables <- lapply(external_full_filenames, FUN=function(file){fread(file, header=T, sep=",")})
}

## ---- calculations
# Now calculate errors
discrete <- data.frame(x_error=c(), y_error=c(), dist_error=c())
discrete$x_error <- gazebo$x_position - discrete$x_position
discrete$y_error <- gazebo$y_position - discrete$y_position
discrete$dist_error <- sqrt(discrete$x_error ^ 2 + discrete$y_error ^ 2)

continuous <- data.frame(x_error=c(), y_error=c(), dist_error=c())
continuous$x_error <- gazebo$x_position - continuous$x_position
continuous$y_error <- gazebo$y_position - continuous$y_position
continuous$dist_error <- sqrt(continuous$x_error ^ 2 + continuous$y_error ^ 2)

external_data_averages <- lapply(external_data_tables, FUN=function(table){table$count[length(table)]})

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

summary(external_data_averages)
