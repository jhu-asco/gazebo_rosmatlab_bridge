#ifndef MMAP_H
#define MMAP_H
//mmap Headers
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>

#if defined(_WIN32) || defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__) || defined(__BORLANDC__)
#define OS_WIN
#include <windows.h>
#endif

#ifndef OS_WIN  
#include <sys/mman.h>
#endif

/** This class provides a way to send and receive data through memory.
		The first byte of the map can be used as a semaphore for synchronizing between client and server
		The next 4 bytes (uint32_t) is used for knowing the length of data
		After that 8 bytes give time stamp; 
*/

template <class T=double>class BufferdMmap {

	protected:
		int fd;
		int n;
		uint8_t *map;
		int32_t readcount;
		int32_t writecount;
		uint32_t writesize, readsize;//Number of bytes to offset to write new message

	public:
		BufferedMmap(const char *fname, uint32_t n)
		{
			int result;
			readcount = 0;//Initializing read buffer count to  zero
			writecount = 0;
			writesize = 0;
			readsize = 0;

			/* Open a file for writing.
			 *  - Creating the file if it doesn't exist.
			 *  - Truncating it to 0 size if it already exists. (not really needed)
			 *
			 * Note: "O_WRONLY" mode is not sufficient when mmaping.
			 */
			
			fd = open(fname, O_RDWR | O_CREAT, (mode_t)0777);
			if (fd == -1) {
				perror("Error opening file for writing");
				return;
			}

			this->n = lseek(fd,0,SEEK_END);
			printf("File Size: %d",this->n);//#DEBUG
			if((this->n) != n)
			{
				this->n = n;

				/* Stretch the file size to the size of the (mmapped) array of ints
				 */
				result = lseek(fd, n*sizeof(uint8_t)-1, SEEK_SET);
				if (result == -1) {
					close(fd);
					perror("Error calling lseek() to 'stretch' the file");
					return;
				}

				/* Something needs to be written at the end of the file to
				 * have the file actually have the new size.
				 * Just writing an empty string at the current file position will do.
				 *
				 * Note:
				 *  - The current position in the file is at the end of the stretched 
				 *    file due to the call to lseek().
				 *  - An empty string is actually a single '\0' character, so a zero-byte
				 *    will be written at the last byte of the file.
				 */
				result = write(fd, "", 1);
				if (result != 1) {
					close(fd);
					perror("Error writing last byte of the file");
					return;
				}
			}

			/* Now the file is ready to be mmapped.
			 */
			map = (uint8_t*)mmap(0, n*sizeof(uint8_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
			if (map == MAP_FAILED) {
				close(fd);
				perror("Error mmapping the file");
				return;
			}

		}

		virtual ~BufferedMmap()
		{
			if (map)
				if (munmap(map, n*sizeof(uint8_t)) == -1)
					perror("Error un-mmapping the file");

			if (fd)
				close(fd);
		}

		uint8_t Status()
		{
			return map[0];
		}

		bool Read( T &out, int32_t &sec, int32_t &nsec)
		{
			if(Status() == 128)//Available to read
			{
				readcount++;
				uint32_t writecount;
				memcpy((uint8_t*)&writecount,(uint8_t*)(map+1),sizeof(uint32_t));//Writecount
				if(readcount >= writecount)//We are not doing actual buffering yet Just simple check
					return false;

				uint32_t serial_size;
				uint8_t *newpointer = (uint8_t *)(map+readsize+5);

				memcpy((uint8_t*)&serial_size,(uint8_t*)(newpointer),sizeof(uint32_t));//Write the size of serial input
				memcpy((uint8_t*)&sec,(uint8_t*)(newpointer+4),sizeof(uint32_t));
				memcpy((uint8_t*)&nsec,(uint8_t*)(newpointer+8),sizeof(uint32_t));
				ros::serialization::IStream stream((uint8_t*)(newpointer+12), serial_size);
				ros::serialization::Serializer<T>::read(stream, out);//Write serial data to stream
				readsize += (12 + serial_size); 
				//map[0] = 0;//Set flag for the the other end to write data
				return true;
			}
			return false;
		}

		bool Read( T &out)
		{
			if(Status() == 128)//Available to read
			{
				readcount++;
				uint32_t writecount;
				memcpy((uint8_t*)&writecount,(uint8_t*)(map+1),sizeof(uint32_t));//Writecount
				if(readcount >= writecount)//We are not doing actual buffering yet Just simple check
					return false;

				uint32_t serial_size;
				uint8_t *newpointer = (uint8_t *)(map+readsize+5);

				memcpy((uint8_t*)&serial_size,(uint8_t*)(newpointer),sizeof(uint32_t));//Write the size of serial input
				ros::serialization::IStream stream((uint8_t*)(newpointer+12), serial_size);
				ros::serialization::Serializer<T>::read(stream, out);//Write serial data to stream
				readsize += (12 + serial_size); 
				//map[0] = 0;//Set flag for the the other end to write data
				return true;
			}
			return false;
		}

		bool Read( T &out, uint32_t &sec, uint32_t &nsec)
		{
			if(Status() == 128)//Available to read
			{
				readcount++;
				uint32_t writecount;
				memcpy((uint8_t*)&writecount,(uint8_t*)(map+1),sizeof(uint32_t));//Writecount
				if(readcount >= writecount)//We are not doing actual buffering yet Just simple check
					return false;

				uint32_t serial_size;
				uint8_t *newpointer = (uint8_t *)(map+readsize+5);

				memcpy((uint8_t*)&serial_size,(uint8_t*)(newpointer),sizeof(uint32_t));//Write the size of serial input
				memcpy((uint8_t*)&sec,(uint8_t*)(newpointer+4),sizeof(uint32_t));
				memcpy((uint8_t*)&nsec,(uint8_t*)(newpointer+8),sizeof(uint32_t));
				ros::serialization::IStream stream((uint8_t*)(newpointer+12), serial_size);
				ros::serialization::Serializer<T>::read(stream, out);//Write serial data to stream
				readsize += (12 + serial_size); 
				//map[0] = 0;//Set flag for the the other end to write data
				return true;
			}
			return false;
		}

		bool Write(const T &in, const int32_t &sec, const int32_t &nsec)
		{
			if(Status() == 0)//Available to write
			{
				writecount++;
				memcpy((uint8_t*)(map+1),(uint8_t*)&writecount,sizeof(uint32_t));//Writecount

				uint32_t serial_size = ros::serialization::serializationLength(in);
				uint8_t *newpointer = (uint8_t *)(map+writesize+5);

				memcpy((uint8_t*)(newpointer),(uint8_t*)&serial_size,sizeof(uint32_t));//Write the size of serial input
				memcpy((uint8_t*)(newpointer+4),(uint8_t*)&sec,sizeof(uint32_t));
				memcpy((uint8_t*)(newpointer+8),(uint8_t*)&nsec,sizeof(uint32_t));
				ros::serialization::OStream model_stream((uint8_t*)(newpointer+12), serial_size);
				ros::serialization::Serializer<T>::write(model_stream, in);//Write serial data to stream
				writesize += (12 + serial_size); 
				//map[0] = 128;//Reset the first bit to read data by the other end
				return true;
			}
			return false;
		}

		bool Write(const T &in, const uint32_t &sec, const uint32_t &nsec)
		{
			if(Status() == 0)//Available to write
			{
				writecount++;
				memcpy((uint8_t*)(map+1),(uint8_t*)&writecount,sizeof(uint32_t));//Writecount

				uint32_t serial_size = ros::serialization::serializationLength(in);
				uint8_t *newpointer = (uint8_t *)(map+writesize+5);

				memcpy((uint8_t*)(newpointer),(uint8_t*)&serial_size,sizeof(uint32_t));//Write the size of serial input
				memcpy((uint8_t*)(newpointer+4),(uint8_t*)&sec,sizeof(uint32_t));
				memcpy((uint8_t*)(newpointer+8),(uint8_t*)&nsec,sizeof(uint32_t));
				ros::serialization::OStream model_stream((uint8_t*)(newpointer+12), serial_size);
				ros::serialization::Serializer<T>::write(model_stream, in);//Write serial data to stream
				writesize += (12 + serial_size); 
				//map[0] = 128;//Reset the first bit to read data by the other end
				return true;
			}
			return false;
		}
};
#endif
