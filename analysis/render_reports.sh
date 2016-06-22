#!/bin/zsh

Rscript -e "rmarkdown::render('one_stationary.R', 'pdf_document')"
Rscript -e "rmarkdown::render('one_mobile.R', 'pdf_document')"

