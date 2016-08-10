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

mean(gps$x)
var(gps$x)
sd(gps$x)

mean(gps$y)
var(gps$y)
sd(gps$y)

mean(gps$horizontal_error)
var(gps$horizontal_error)
sd(gps$horizontal_error)
