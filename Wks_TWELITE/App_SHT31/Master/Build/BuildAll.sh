#!/bin/sh

make TWELITE=BLUE APP_UART_CONFIG=CONFIG_NORMAL clean
make TWELITE=BLUE APP_UART_CONFIG=CONFIG_NORMAL all
make TWELITE=BLUE APP_UART_CONFIG=CONFIG_NORMAL TWE_DEVKIT=MONOSTICK clean
make TWELITE=BLUE APP_UART_CONFIG=CONFIG_NORMAL TWE_DEVKIT=MONOSTICK all

make TWELITE=RED APP_UART_CONFIG=CONFIG_NORMAL clean
make TWELITE=RED APP_UART_CONFIG=CONFIG_NORMAL all
make TWELITE=RED APP_UART_CONFIG=CONFIG_NORMAL TWE_DEVKIT=MONOSTICK clean
make TWELITE=RED APP_UART_CONFIG=CONFIG_NORMAL TWE_DEVKIT=MONOSTICK all