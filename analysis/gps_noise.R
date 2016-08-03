if (!require("data.table")){
    install.packages("data.table", repos="http://cran.rstudio.com/")
    library("data.table")
}

file_name <- "/home/matt/thesis/experiment_data/one_stationary/turtlebot1_gps_data.csv"
gps <- fread(file_name, header=T, sep=",")

plot(gps$x, gps$y, main="Sampled Odom Particles")
gps$horizontal_error <- sqrt(gps$x ^ 2 + gps$y ^ 2) * sample(c(-1, 1), 2)

hist(gps$horizontal_error)
shapiro.test(gps$horizontal_error)
