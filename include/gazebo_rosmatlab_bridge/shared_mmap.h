#ifndef SHAREDMMAP_H
#define SHAREDMMAP_H
//mmap Headers
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(_WIN32) || defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__) || defined(__BORLANDC__)
#define OS_WIN
#include <windows.h>
#endif

#ifndef OS_WIN  
#include <sys/mman.h>
#endif

/** This class provides a way to send and receive data through memory.
		The first byte of the map can be used as a semaphore for synchronizing between client and server
		The second 4 bytes (uint32_t) is used for knowing the length of data
*/

template <class T=double>class SharedMmap {

	protected:
		int fd;
		int n;
		T *map;

	public:
		SharedMmap(const char *fname, int n)
		{
			int result;

			/* Open a file for writing.
			 *  - Creating the file if it doesn't exist.
			 *  - Truncating it to 0 size if it already exists. (not really needed)
			 *
			 * Note: "O_WRONLY" mode is not sufficient when mmaping.
			 */
			 if(n != 0)
			 {
				 fd = open(fname, O_RDWR | O_CREAT | O_TRUNC, (mode_t)0777);
				 if (fd == -1) {
					 perror("Error opening file for writing");
					 return;
				 }
			 }
			 else
			 {
				 fd = open(fname, O_RDWR | O_CREAT, (mode_t)0777);
				 if (fd == -1) {
					 perror("Error opening file for reading");
					 return;
				 }
			 }

			if(n == 0)
			{
				this->n = lseek(fd,0,SEEK_END);
			}
			else
			{
				this->n = 30 + n*sizeof(T);//Just for more buffer
			}
			printf("Number of bytes: %d",this->n);

			/* Stretch the file size to the size of the (mmapped) array of ints
			 */
			result = lseek(fd, (this->n)*sizeof(uint8_t), SEEK_SET); //1+4*3 + 4*2 -1
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

			/* Now the file is ready to be mmapped.
			 */
			map = (uint8_t*)mmap(0, (this->n)*sizeof(uint8_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
			if (map == MAP_FAILED) {
				close(fd);
				perror("Error mmapping the file");
				return;
			}
		}

		virtual ~SharedMmap()
		{
			if (map)
				if (munmap(map, n*sizeof(uint8_t)) == -1)
					perror("Error un-mmapping the file");

			if (fd)
				close(fd);
		}

		uint8_t Status()//Use this to decide whether to read[true] or write[false] data
		{
			return map[0];
		}

		bool Read(T *basepointer, uint32_t *nofelems)
		{
			if(Status() != 128)//Not ready to read
				return false;
			memcpy((uint8_t *)basepointer,(uint8_t *)(map + 21),nofelems[0]*nofelems[1]*nofelems[2]*sizeof(T));
			map[0] = 0;//Ready to write
			return true;
		}

		bool Read(T *basepointer, uint32_t *nofelems, int32_t &sec, int32_t &nsec)
		{
			if(Status() != 128)//Not ready to read
				return false;
			memcpy((uint8_t *)&sec,(uint8_t*)(map+13), sizeof(uint32_t));//Read 4 bytes of data
			memcpy((uint8_t *)&nsec,(uint8_t*)(map+17), sizeof(uint32_t));//Read 4 bytes of data
			memcpy((uint8_t *)basepointer,(uint8_t *)(map + 21),nofelems[0]*nofelems[1]*nofelems[2]*sizeof(T));
			map[0] = 0;//Ready to write
			return true;
		}
		
		bool Read(T *basepointer, uint32_t *nofelems, uint32_t &sec, uint32_t &nsec)
		{
			if(Status() != 128)//Not ready to read
				return false;
			memcpy((uint8_t *)&sec,(uint8_t*)(map+13), sizeof(uint32_t));//Read 4 bytes of data
			memcpy((uint8_t *)&nsec,(uint8_t*)(map+17), sizeof(uint32_t));//Read 4 bytes of data
			memcpy((uint8_t *)basepointer,(uint8_t *)(map + 21),nofelems[0]*nofelems[1]*nofelems[2]*sizeof(T));
			map[0] = 0;//Ready to write
			return true;
		}

		bool Readnofelems(uint32_t *nofelems)
		{
			if(Status() != 128)//Not ready to read
				return false;
			memcpy((uint8_t *)nofelems,(uint8_t*)(map+1), 3*sizeof(uint32_t));//Read 4 bytes of data
			return true;
		}
		bool Write(const T *basepointer,const uint32_t *nofelems, const uint32_t &sec, const uint32_t &nsec)
		{
			if(Status() != 0)//Not ready to write
				return false;
			memcpy((uint8_t*)(map+1),(uint8_t*)nofelems,3*sizeof(uint32_t));
			memcpy((uint8_t*)(map+13),(uint8_t*)&sec,sizeof(uint32_t));
			memcpy((uint8_t*)(map+17),(uint8_t*)&nsec,sizeof(uint32_t));
			memcpy((uint8_t*)(map+21),(uint8_t*)basepointer,nofelems[0]*nofelems[1]*nofelems[2]*sizeof(T));
			map[0] = 128;
			return true;
		}

		bool Write(const T *basepointer,const uint32_t *nofelems)
		{
			if(Status() != 0)//Not ready to write
				return false;
			memcpy((uint8_t*)(map+1),(uint8_t*)nofelems,3*sizeof(uint32_t));
			memcpy((uint8_t*)(map+21),(uint8_t*)basepointer,nofelems[0]*nofelems[1]*nofelems[2]*sizeof(T));
			map[0] = 128;
			return true;
		}
		bool Write(const T *basepointer,const uint32_t *nofelems, const int32_t &sec, const int32_t &nsec)
		{
			if(Status() != 0)//Not ready to write
				return false;
			memcpy((uint8_t*)(map+1),(uint8_t*)nofelems,3*sizeof(uint32_t));
			memcpy((uint8_t*)(map+13),(uint8_t*)&sec,sizeof(uint32_t));
			memcpy((uint8_t*)(map+17),(uint8_t*)&nsec,sizeof(uint32_t));
			memcpy((uint8_t*)(map+21),(uint8_t*)basepointer,nofelems[0]*nofelems[1]*nofelems[2]*sizeof(T));
			map[0] = 128;//Ready to read
			return true;
		}
};
#endif
