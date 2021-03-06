#!/usr/bin/env Rscript

data_dir <- "/home/matt/thesis/experiment_data"
dirs <- list.dirs(path=data_dir, recursive=FALSE) # Find all experiments in the data_dir
report_dir <- paste0(data_dir, "/reports") # Create a subdirectory in the data_dir to store reports
args <- commandArgs(trailingOnly = TRUE)

renderAll <- function() {
    message("No args received, rendering all experiments")
    for (directory in 1:length(dirs)){
        experiment_name = substr(dirs[directory], nchar(data_dir)+2, nchar(dirs[directory]))
        files <- list.files(path=dirs[directory], pattern="turtlebot([0-9])+_gazebo_odom.csv")

        if (experiment_name != "reports" && experiment_name != "old") {
            message(paste0("Rendering experiment ", experiment_name))
            for (file in 1:length(files)) {
                if (file < 10) {
                    robot_number <- substr(files[file], 10, 10)
                }
                else if (file < 100) {
                    robot_number <- substr(files[file], 10, 11)
                }

                rmarkdown::render("robot.Rmd",
                                  params=list(experiment=experiment_name, robot=robot_number),
                                  output_file=paste0("turtlebot_", robot_number, ".pdf"),
                                  output_dir=paste(report_dir, experiment_name, sep="/"),
                                  quiet=TRUE)
            }

            rmarkdown::render('experiment.Rmd',
                              params=list(experiment=experiment_name, robots=length(files)),
                              output_file=paste0(experiment_name, ".pdf"),
                              output_dir=paste(report_dir, experiment_name, sep="/"),
                              quiet=TRUE)
        }
    }
}

renderSome <- function() {
    message("Some args received, only rendering specified experiments")
    for (directory in 1:length(dirs)){
        experiment_name = substr(dirs[directory], nchar(data_dir)+2, nchar(dirs[directory]))
        files <- list.files(path=dirs[directory], pattern="turtlebot([0-9])+_gazebo_odom.csv")

        if (is.element(experiment_name, args)) {
            message(paste0("Rendering experiment ", experiment_name))
            for (file in 1:length(files)) {
                if (file < 10) {
                    robot_number <- substr(files[file], 10, 10)
                }
                else if (file < 100) {
                    robot_number <- substr(files[file], 10, 11)
                }

                rmarkdown::render("robot.Rmd",
                                  params=list(experiment=experiment_name, robot=robot_number),
                                  output_file=paste0("turtlebot_", robot_number, ".pdf"),
                                  output_dir=paste(report_dir, experiment_name, sep="/"),
                                  quiet=TRUE)
            }

            rmarkdown::render('experiment.Rmd', params=list(experiment=experiment_name, robots=length(files)),
                              output_file=paste0(experiment_name, ".pdf"),
                              output_dir=paste(report_dir, experiment_name, sep="/"),
                              quiet=TRUE)
        }
    }
}

message('Ready to analyze experiment data!')
# Accept command line args to control if we render all reports, or just for some experiments

if (length(args) == 0) {
    # If no args, render everything that we can
    renderAll()
} else {
    # If some args, render only experiments with names matching the passed args
    renderSome()
}
