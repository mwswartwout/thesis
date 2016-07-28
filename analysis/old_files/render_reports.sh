#!/bin/zsh

Rscript -e "rmarkdown::render('one_robot.Rmd', params=list(experiment_name='one_stationary'), output_file='stationary.pdf')"
Rscript -e "rmarkdown::render('one_robot.Rmd', params=list(experiment_name='one_mobile'), output_file='mobile.pdf')"

