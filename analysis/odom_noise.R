if (!require("data.table")){
    install.packages("data.table", repos="http://cran.rstudio.com/")
    library("data.table")
}

file_name <- "/home/matt/thesis/ros_ws/src/cwru_turtlebot/src/cwru_turtlebot/odom_noise_test.csv"
odom <- fread(file_name, header=T, sep=",")

plot(odom$x, odom$y, main="Sampled Odom Particles", sub="Noise parameters: [0.2, 0.2, 0.2, 0.2]", xlim=c(0, 1.5), ylim=c(0, 1.5))
