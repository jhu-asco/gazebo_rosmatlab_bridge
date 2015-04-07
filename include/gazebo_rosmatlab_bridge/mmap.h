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
		The second 4 bytes (uint32_t) is used for knowing the length of data
*/

template <class T=double>class Mmap {

	protected:
		int fd;
		int n;
		uint8_t *map;

	public:
		Mmap(const char *fname, int n)
		{
			int result;

			/* Open a file for writing.
			 *  - Creating the file if it doesn't exist.
			 *  - Truncating it to 0 size if it already exists. (not really needed)
			 *
			 * Note: "O_WRONLY" mode is not sufficient when mmaping.
			 */
      if(n!= 0)
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
        if(this->n <=0)
        {
          printf("Size of file is less than or equal to 0");
          close(fd);
          return;
        }
      }
      else
      {
        this->n = n;
      }

			/* Stretch the file size to the size of the (mmapped) array of ints
			 */
			result = lseek(fd, (this->n)*sizeof(uint8_t)-1, SEEK_SET);
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
      if(n != 0)
      {
        result = write(fd, "", 1);
        if (result != 1) {
          close(fd);
          perror("Error writing last byte of the file");
          return;
        }
      }

			/* Now the file is ready to be mmapped.
			 */
			map = (uint8_t*)mmap(0, (this->n)*sizeof(uint8_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
			if (map == MAP_FAILED) {
				close(fd);
				perror("Error mmapping the file");
				return;
			}
      //Set the map mode to write mode whenever map constructor is invoked for that stream.
      //This will ensure that new data will be written by the server when client is created.
      map[0] = 0;
		}

		virtual ~Mmap()
		{
      if(this->n <=0)//Nothing to do
        return;
      map[0] = 0;//Set to write mode before closing
			if (map)
				if (munmap(map, n*sizeof(uint8_t)) == -1)
					perror("Error un-mmapping the file");

			if (fd)
				close(fd);
		}

		uint8_t Status()
		{
      if(fd <=0)
        return 2;//Not available to read or write since the stream could not be opened
			return map[0];
		}

		/*uint8_t *Data() const
		{
			return map;
		}
		*/

		bool Read( T &out, int32_t &sec, int32_t &nsec)
		{
			if(Status() == 128)//Available to read
			{
				uint32_t serial_size;
				memcpy((uint8_t *)&serial_size,(uint8_t*)(map+1), sizeof(uint32_t));//Read 4 bytes of data
				memcpy((uint8_t *)&sec,(uint8_t*)(map+5), sizeof(uint32_t));//Read 4 bytes of data
				memcpy((uint8_t *)&nsec,(uint8_t*)(map+9), sizeof(uint32_t));//Read 4 bytes of data
				ros::serialization::IStream stream((uint8_t *)(map+13), serial_size);
				ros::serialization::Serializer<T>::read(stream, out);
				map[0] = 0;//Set flag for the the other end to write data
				return true;
			}
			return false;
		}

		bool Read( T &out)
		{
			if(Status() == 128)//Available to read
			{
				uint32_t serial_size;
				memcpy((uint8_t *)&serial_size,(uint8_t*)(map+1), sizeof(uint32_t));//Read 4 bytes of data
				ros::serialization::IStream stream((uint8_t *)(map+13), serial_size);
				ros::serialization::Serializer<T>::read(stream, out);
				map[0] = 0;//Set flag for the the other end to write data
				return true;
			}
			return false;
		}
		bool Read( T &out, uint32_t &sec, uint32_t &nsec)
		{
			if(Status() == 128)//Available to read
			{
				uint32_t serial_size;
				memcpy((uint8_t *)&serial_size,(uint8_t*)(map+1), sizeof(uint32_t));//Read 4 bytes of data
				memcpy((uint8_t *)&sec,(uint8_t*)(map+5), sizeof(uint32_t));//Read 4 bytes of data
				memcpy((uint8_t *)&nsec,(uint8_t*)(map+9), sizeof(uint32_t));//Read 4 bytes of data
				ros::serialization::IStream stream((uint8_t *)(map+13), serial_size);
				ros::serialization::Serializer<T>::read(stream, out);
				map[0] = 0;//Set flag for the the other end to write data
				return true;
			}
			return false;
		}

		bool Write(const T &in)
		{
			if(Status() == 0)//Available to write
			{
				uint32_t serial_size = ros::serialization::serializationLength(in);
				memcpy((uint8_t*)(map+1),(uint8_t*)&serial_size,sizeof(uint32_t));//Write the size of serial input
				ros::serialization::OStream model_stream((uint8_t*)(map+13), serial_size);
				ros::serialization::Serializer<T>::write(model_stream, in);//Write serial data to stream
				map[0] = 128;//Reset the first bit to read data by the other end
				return true;
			}
			return false;
		}

		bool Write(const T &in, const int32_t &sec, const int32_t &nsec)
		{
			if(Status() == 0)//Available to write
			{
				uint32_t serial_size = ros::serialization::serializationLength(in);
				memcpy((uint8_t*)(map+1),(uint8_t*)&serial_size,sizeof(uint32_t));//Write the size of serial input
				memcpy((uint8_t*)(map+5),(uint8_t*)&sec,sizeof(uint32_t));
				memcpy((uint8_t*)(map+9),(uint8_t*)&nsec,sizeof(uint32_t));
				ros::serialization::OStream model_stream((uint8_t*)(map+13), serial_size);
				ros::serialization::Serializer<T>::write(model_stream, in);//Write serial data to stream
				map[0] = 128;//Reset the first bit to read data by the other end
				return true;
			}
			return false;
		}

		bool Write(const T &in, const uint32_t &sec, const uint32_t &nsec)
		{
			if(Status() == 0)//Available to write
			{
				uint32_t serial_size = ros::serialization::serializationLength(in);
				memcpy((uint8_t*)(map+1),(uint8_t*)&serial_size,sizeof(uint32_t));//Write the size of serial input
				memcpy((uint8_t*)(map+5),(uint8_t*)&sec,sizeof(uint32_t));
				memcpy((uint8_t*)(map+9),(uint8_t*)&nsec,sizeof(uint32_t));
				ros::serialization::OStream model_stream((uint8_t*)(map+13), serial_size);
				ros::serialization::Serializer<T>::write(model_stream, in);//Write serial data to stream
				map[0] = 128;//Reset the first bit to read data by the other end
				return true;
			}
			return false;
		}
};
#endif
