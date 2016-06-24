data_dir <- "/home/matt/thesis/experiment_data"
dirs <- list.dirs(path=data_dir)
report_dir <- paste0(data_dir, "/reports")

# First let's delete any previous reports, as having those around seems to cause errors
unlink(paste(report_dir, "*", sep="/"), recursive=TRUE)

for (directory in 2:length(dirs)){
    experiment_name = substr(dirs[directory], nchar(data_dir)+2, nchar(dirs[directory]))
    files <- list.files(path=dirs[directory], pattern="turtlebot([0-9])+_gazebo_odometry_filtered.csv")

    if (experiment_name != "reports") {
        for (file in 1:length(files)) {
            if (file < 10) {
                robot_number <- substr(files[file], 10, 10)
            }
            else if (file < 100) {
                robot_number <- substr(files[file], 10, 11)
            }

            rmarkdown::render("robot.Rmd", params=list(experiment=experiment_name, robot=robot_number), output_file=paste0("turtlebot_", robot_number, ".pdf"), output_dir=paste(report_dir, experiment_name, sep="/"))
        }

        rmarkdown::render('experiment.Rmd', params=list(experiment=experiment_name, robots=length(files)), output_file=paste0(experiment_name, ".pdf"), output_dir=paste(report_dir, experiment_name, sep="/"))
    }


}
