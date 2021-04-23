#!/bin/bash

sudo apt update

sudo apt install tesseract-ocr

pip3 install pytesseract

result=$(tesseract --version | grep "tesseract 4")
if [ -n "$result" ]
then
    echo "Successfully installed."
else
    echo "Failed to install."
fi