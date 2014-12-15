#include <flann/flann.hpp>
#include <iostream>
#include <cstdio>

int main(int argc, char** argv)
{
	int nn = 3;

	float rawDataset[] = {1, 1, 1,
						  1, 1, -1,
						  1, -1, -1,
						  1, -1, 1,
						  -1, -1, 1,
						  -1, -1, -1,
						  -1, 1, -1,
						  -1, 1, 1};

	float rawQueries[] = {1, 1, 1,
						  -1, -1, -1};

	flann::Matrix<float> dataset(rawDataset, 8, 3);
	flann::Matrix<float> query(rawQueries, 2, 3);

	flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
	flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);

	flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
	index.buildIndex();

	index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));

	std::cout << "Nearest Neighbors Indices" << std::endl;
	for(int i=0; i<indices.rows; i++)
	{
		printf("%d query indices: ", (int)rawQueries[i]);
		for(int j=0; j<indices.cols; j++)
		{
			if(j != 0)
				std::cout << ", ";
			std::cout << indices[i][j];
		}
		std::cout << std::endl;
	}

	std::cout << "Nearest Neighbors Distances" << std::endl;
	for(int i=0; i<dists.rows; i++)
	{
		for(int j=0; j<dists.cols; j++)
		{
			if(j != 0)
				std::cout << ", ";
			std::cout << dists[i][j];
		}
		std::cout << std::endl;
	}
	
	//flann::save_to_file(indices,"result.hdf5","result");
	delete[] indices.ptr();
	delete[] dists.ptr();
	return 0;
	
						  
	
}
