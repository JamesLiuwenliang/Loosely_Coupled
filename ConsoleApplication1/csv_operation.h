#pragma once
#ifndef CSV_OPERATION_H
#define CSV_OPERATION_H

void print_two_dimension(double** data, int row, int col);
void get_two_dimension(char* line, double** data, char *filename);
int get_col(char *filename);
int get_row(char *filename);
void get_one_line(char* line, char *filename, int row_now);

#endif