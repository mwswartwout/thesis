data_dir <- "/home/matt/thesis/experiment_data"
dirs <- list.dirs(path=data_dir)

for (i in 2:length(dirs)){
    experiment_name = substr(dirs[i], nchar(data_dir)+2, nchar(dirs[i]))
    if (substr(experiment_name, 1, 4) == "one_") {
        rmarkdown::render("one_robot.Rmd", params=list(experiment=experiment_name), output_file=paste0(experiment_name, ".pdf"), output_dir=paste0(data_dir, "/reports/"))
    }
    else {
        message("Found a directory which cannot be processed: ", dirs[i])
    }
}

