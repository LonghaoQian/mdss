#include "pch.h"
#include "util_Recorder.h"


util_Recorder::util_Recorder(const char* _filename, unsigned int _num_of_colums)
{
	filename = _filename;
	numbe_of_column = _num_of_colums;
	file.open(filename, std::ofstream::out);
	file.close();
	data_counter = 0;
}

util_Recorder::~util_Recorder()
{
	file.close();
}

void util_Recorder::ResetRecorder()
{
	data_counter = 0;
}
void util_Recorder::Record(double* data)
{
	if (!file.is_open())
	{
		file.open(filename, std::ofstream::out);
	}
	data_counter++;
	file << data_counter;
	for (int i = 0; i < numbe_of_column; i++)
	{
		file << "XX" << data[i];
	}
	file << endl;

}
