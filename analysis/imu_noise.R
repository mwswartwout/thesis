if (!require("data.table")){
    install.packages("data.table", repos="https://cran.rstudio.com/")
    library("data.table")
}

if (!require("fitdistrplus")){
    install.packages("fitdistrplus", repos="https://cran.rstudio.com/")
    library("fitdistrplus")
}

file_name <- "/home/matt/thesis/experiment_data/one_mobile_noisy_true/turtlebot1_raw_imu_data.csv"
imu <- fread(file_name, header=T, sep=",")

summary(imu$yaw)
summary(imu$x_vel)
summary(imu$z_vel)
summary(imu$x_acc)

shapiro.test(imu$yaw)
shapiro.test(imu$x_vel)
shapiro.test(imu$z_vel)
shapiro.test(imu$x_acc)

plot(imu$yaw)
plot(imu$x_vel)
plot(imu$z_vel)
plot(imu$x_acc)

hist(imu$yaw)
hist(imu$x_vel)
hist(imu$z_vel)
hist(imu$x_acc)

plot(density(imu$x_vel))
plot(density(imu$z_vel))
plot(density(imu$x_acc))

plot(ecdf(imu$x_vel))
plot(ecdf(imu$z_vel))
plot(ecdf(imu$x_acc))

qqnorm(imu$x_vel)
abline(0,1)
qqnorm(imu$z_vel)
abline(0,1)
qqnorm(imu$x_acc)
abline(0,1)

x_vel_std <- (imu$x_vel - mean(imu$x_vel)) / sd(imu$x_vel)
qqnorm(x_vel_std)
abline(0,1)
z_vel_std <- (imu$z_vel - mean(imu$z_vel)) / sd(imu$z_vel)
qqnorm(z_vel_std)
abline(0,1)
x_acc_std <- (imu$x_acc - mean(imu$x_acc)) / sd(imu$x_acc)
qqnorm(x_acc_std)
abline(0,1)

#shapiro.test(imu$yaw)
shapiro.test(x_vel_std)
shapiro.test(z_vel_std)
shapiro.test(x_acc_std)

# plot(imu$yaw, main="IMU Yaw Readings", sub="Stationary")
# plot(imu$x_vel, main="IMU X Velocity Readings", sub="Stationary")
# plot(imu$y_vel, main="IMU Y Velocity Readings", sub="Stationary")
# plot(imu$z_vel, main="IMU Z Velocity Readings", sub="Stationary")
# plot(imu$x_acc, main="IMU X Acceleration Readings", sub="Stationary")
# plot(imu$y_acc, main="IMU Y Acceleration Readings", sub="Stationary")
# plot(imu$z_acc, main="IMU Z Acceleration Readings", sub="Stationary")
#
# yaw <- fitdist(imu$yaw, "norm", method="mme")
# x_vel <- fitdist(imu$x_vel, "norm", method="mme")
# y_vel <- fitdist(imu$y_vel, "norm", method="mme")
# z_vel <- fitdist(imu$z_vel, "norm", method="mme")
# x_acc <- fitdist(imu$x_acc, "norm", method="mme")
# y_acc <- fitdist(imu$y_acc, "norm", method="mme")
# z_acc <- fitdist(imu$z_acc, "norm", method="mme")
#
# summary(yaw)
# summary(x_vel)
# summary(y_vel)
# summary(z_vel)
# summary(x_acc)
# summary(y_acc)
# summary(z_acc)
#
# plot(yaw)
# plot(x_vel)
# plot(y_vel)
# plot(z_vel)
# plot(x_acc)
# plot(y_acc)
# plot(z_acc)
