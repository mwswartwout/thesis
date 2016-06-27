if (!require("data.table")){
    install.packages("data.table", repos="http://cran.rstudio.com/")
    library("data.table")
}

# First we need to get data from each robot and combine into one data set
## ---- read_data
for (robot in 1:params$robots) {
    gazebo_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_gazebo_odometry_filtered.csv")})
    gazebo_full_filenames <- lapply(gazebo_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    gazebo <- rbindlist(lapply(gazebo_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    discrete_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_discrete_odometry_filtered.csv")})
    discrete_full_filenames <- lapply(discrete_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    discrete <- rbindlist(lapply(discrete_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    continuous_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_continuous_odometry_filtered.csv")})
    continuous_full_filenames <- lapply(continuous_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    continuous <- rbindlist(lapply(continuous_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    external_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_external_pose_count.csv")})
    external_full_filenames <- lapply(external_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    external_data_tables <- lapply(external_full_filenames, FUN=function(file){fread(file, header=T, sep=",")})
}

## ---- calculations
# Now calculate errors
gazebo$dist_from_origin <- sqrt(gazebo$x_position ^ 2 + gazebo$y_position ^ 2)

discrete$x_error <- gazebo$x_position - discrete$x_position
discrete$y_error <- gazebo$y_position - discrete$y_position
discrete$dist_error <- sqrt(discrete$x_error ^ 2 + discrete$y_error ^ 2)

continuous$x_error <- gazebo$x_position - continuous$x_position
continuous$y_error <- gazebo$y_position - continuous$y_position
continuous$dist_error <- sqrt(continuous$x_error ^ 2 + continuous$y_error ^ 2)

external_data_averages <- lapply(external_data_tables, FUN=function(table){table$count[length(table)]})

## ---- plot
plot(gazebo$x_position, gazebo$y_position,
     main = "Ground truth visited locations of robot")

hist(gazebo$dist_from_origin,
     main = "Distance from origin vs. time")

hist(continuous$x_error,
     main = "Continuous x_error")

hist(continuous$y_error,
      main = "Continuous y_error")

hist(continuous$dist_error,
     main = "Continuous total distance error")

hist(discrete$x_error,
     main = "Discrete x_error")

hist(discrete$y_error,
     main = "Discrete y_error")

hist (discrete$dist_error,
      main = "Discrete total distance error")

## ---- summary
summary(continuous$x_error)
summary(continuous$y_error)
summary(continuous$dist_error)

summary(discrete$x_error)
summary(discrete$y_error)
summary(discrete$dist_error)

if (params$robot >= 2) {
    summary(external_data_averages)
}
