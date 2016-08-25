if (!require(ggplot2)){
  install.packages("ggplot2", repos="http://cran.rstudio.com/")
  library(ggplot2)
}

# Discrete Filter
df <- data.frame(
                experiment = c("Stationary Individual", "Mobile Individual", "Mobile Group"),
                means = c(.26, 1.366, .954),
                sds = c(.199, 1.019, .663),
                maxes = c(.633, 6.037, 4.213),
                mins = c(.000001, .00001, .00002))

df$experiment <- factor(df$experiment, levels = df$experiment)

ggplot(df, aes(x=experiment)) +
  geom_errorbar(aes(ymax = means + sds, ymin = means -sds), position="dodge") +
  ggtitle("Distributed Filter Position Error Summary\n
          mean +/- std. dev., max and min error") +
  ylab("Position error (m)") +
  geom_point(aes(y = maxes)) +
  geom_point(aes(y = mins))

