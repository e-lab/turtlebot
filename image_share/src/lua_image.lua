--Reference: https://github.com/jtbates/lua---camera/blob/master/opencv/opencv.c

require 'inline'
require 'image'

inline.preamble[[
	#include<sys/stat.h>
	#include<sys/mman.h>
	#include<fcntl.h>
	#include<unistd.h>	

	typedef struct image_data{
		unsigned char data[640*480*3];
	}image_data;

	typedef struct depth_data{
		short int data[640*480*3];
	}depth_data;
]]
image_convert = inline.load[[
	image_data* mapped_ptr;
	int fd = shm_open("/image_share", (O_RDWR), (S_IRUSR|S_IWUSR));
	
	THFloatTensor* float_tensor = luaT_checkudata(L,1,luaT_checktypename2id(L,"torch.FloatTensor"));
	float *c_tensor = THFloatTensor_data(float_tensor);
	
	int width_inc = float_tensor->stride[2];
	int height_inc = float_tensor->stride[1];
	int dim_inc = float_tensor->stride[0];
	lockf(fd,F_LOCK,sizeof(image_data));
	mapped_ptr = (image_data*)mmap(0, sizeof(image_data), (PROT_READ|PROT_WRITE), MAP_SHARED, fd, 0);
	int i,j,num;
	for(i=0; i<480; i++){
		for(j=0; j<640; j++){
			num = i*height_inc+j*width_inc;
			c_tensor[num] = ((unsigned int)(mapped_ptr->data[i*(3*640)+j*3]))/255.0;
			c_tensor[dim_inc + num] = ((unsigned int)(mapped_ptr->data[i*(3*640)+j*3+1]))/255.0;
			c_tensor[dim_inc*2 + num] = ((unsigned int)(mapped_ptr->data[i*(3*640)+j*3+2]))/255.0;
		}
	}
	lockf(fd,F_ULOCK,sizeof(image_data));
]]

depth_convert = inline.load[[
	depth_data* mapped_ptr;
	int fd = shm_open("/image_share", (O_RDWR), (S_IRUSR|S_IWUSR));
	
	THFloatTensor* float_tensor = luaT_checkudata(L,1,luaT_checktypename2id(L,"torch.FloatTensor"));
	float *c_tensor = THFloatTensor_data(float_tensor);
	
	int width_inc = float_tensor->stride[2];
	int height_inc = float_tensor->stride[1];
	int dim_inc = float_tensor->stride[0];
	lockf(fd,F_LOCK,sizeof(depth_data));
	mapped_ptr = (depth_data*)mmap(0, sizeof(depth_data), (PROT_READ|PROT_WRITE), MAP_SHARED, fd, 0);
	int i,j,num;
	for(i=0; i<480; i++){
		for(j=0; j<640; j++){
			num = i*height_inc+j*width_inc;
			c_tensor[num] = ((unsigned int)(mapped_ptr->data[i*(3*640)+j*3]))/255.0;
			c_tensor[dim_inc + num] = ((unsigned int)(mapped_ptr->data[i*(3*640)+j*3+1]))/255.0;
			c_tensor[dim_inc*2 + num] = ((unsigned int)(mapped_ptr->data[i*(3*640)+j*3+2]))/255.0;
		}
	}
	lockf(fd,F_ULOCK,sizeof(depth_data));
]]
a=torch.FloatTensor(3,480,640)
image_convert(a)
image.display(a)
--b=torch.FloatTensor(3,480,640)
--depth_convert(b)
--image.display(b)
