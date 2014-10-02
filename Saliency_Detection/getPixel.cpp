unsigned int *input = (unsigned int*)(fgy.data);
	int i,j,r,g,b;
	for(int i = 0;i < fgy.cols;i++){
		for(int j = 0;j < fgy.rows;j++){
			if( input[fgy.cols * j + i] )
				cout << j << endl;
			
		}
	}
