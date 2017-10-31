#include <sys/statvfs.h>
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <iostream>   
#include <cstdlib> 
#include <math.h>

using namespace std;

int main( int argc, char *argv[] )
{
    struct statvfs fiData;
	char* storage_name;
	storage_name = (char*)malloc(200 * sizeof(char));
	if(storage_name == NULL){
		//cerr << "Error..." << endl;
		return 0;
	}
	strcpy(storage_name,"/media/ubuntu/3163-6234/"); 
	//cout<<storage_name<<endl;
	
    //Lets loopyloop through the argvs
	if((statvfs(storage_name,&fiData)) < 0 ) {
			cout << "\nFailed to stat:"  << endl;
			return 0;
	} else {
		//cout << "\nDisk: " <<  argv[i];
/*			cout << "\nBlock size: "<< fiData.f_bsize;
			cout << "\nTotal no blocks: "<< fiData.f_blocks;
			cout << "\nFree blocks: "<< fiData.f_bfree;*/
			
			//if (fiData.f_bsize*fiData.f_bfree<3)
			float free_space = (long long)(fiData.f_bfree)*(long long)fiData.f_bsize * pow(10,-9);
			//cout << free_space << endl;	
			if (free_space<0.1)	return 0;
	}
	
	return 1;
    
}