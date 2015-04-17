#include "mesh.h"
#include <cmath>
#include "utils.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace gcop;
using namespace std;

/*Mesh::Mesh(const Mesh &mesh) :
  w(mesh.w), h(mesh.h), cs(mesh.cs), ds(mesh.ds), ni(mesh.ni), nj(mesh.nj), eps(mesh.eps)
{
  memcpy(o, mesh.o, 3*sizeof(double));
  data = new double[nj*ni];
  memcpy(data, mesh.data, nj*ni*sizeof(double));
  odata = 0;
  normals = new double[nj*ni*3];
  memcpy(normals, mesh.normals, nj*ni*3*sizeof(double));
}
*/

Mesh::Mesh() : w(0), h(0), cs(0), ds(0), ni(0), nj(0), data(0), odata(0), normals(0), eps(1e-10)
{
  memset(this->o, 0, 3*sizeof(double));
}

/*Mesh::Mesh(double w, double h, double cs, double ds, const double *o) :
  w(w), h(h), cs(cs), ds(ds), eps(1e-10)
{
  if (o)
    memcpy(this->o, o, 3*sizeof(double));
  else
    memset(this->o, 0, 3*sizeof(double));
  nj = (int)(w/cs+1);
  ni = (int)(h/cs+1);
  data = new double[nj*ni];
  odata = 0;
  normals = new double[nj*ni*3];
}
*/


Mesh::Mesh(const char *fname, const double *o, const double *scale) :
  w(0), h(0), eps(1e-10)
{
  //Set origin
  if (o)
    memcpy(this->o, o, 3*sizeof(double));
  else
    memset(this->o, 0, 3*sizeof(double));

  //Set scale
  if (scale)
    memcpy(this->scale, scale, 3*sizeof(double));
  else
    memset(this->scale, 0, 3*sizeof(double));
  Load(fname);
  odata = 0;

  ComputeNormals();
}


Mesh::~Mesh()
{
  delete[] normals;
  delete[] data;
  delete[] odata;
}

const double* Mesh::GetNormal(int i, int j) const
{
  return normals + 3*(j + nj*i);
}

double Mesh::bilinterp(const double* z, int w, int h, double xi, double yi, double eps)
{
  double t,u;
  int x1, x2, y1, y2;
  
  //  cout << "xi=" << xi << " yi=" << yi << endl;

  if (xi < 0 && xi > -eps)
    xi = 0;
  if (xi > w - 1 && xi < w - 1 + eps)
    xi = w - 1 - eps;

  if (yi < 0 && yi > -eps)
    yi = 0;
  if (yi > h - 1 && yi < h - 1 + eps )
    yi = h - 1 - eps;

  x1 = (int)floor(xi);
  y1 = (int)floor(yi);
  x2 = (int)ceil(xi);
  y2 = (int)ceil(yi);


  if (x1 < 0 || x1 >= w || y1 < 0 || y1 >= h) {     
    fprintf(stderr, "Error - mesh:bilinterp - invalid (x1,y1)=(%d,%d) (w=%d, h=%d)\n", x1,y1,w, h);
    return 0;
  }
  if (x2 < 0 || x2 >= w || y2 < 0 || y2 >= h) {
    fprintf(stderr, "Error - mesh:bilinterp - invalid (x2,y2)=(%d,%d) (w=%d, h=%d)\n", x2,y2, w, h);
    return 0;
  }

  t = (xi - x1);
  u = (yi - y1);

  return (1-t)*(1-u)*z[y1*w + x1] + t*(1-u)*z[y1*w + x2] + t*u*z[y2*w + x2] + (1-t)*u*z[y2*w + x1];
}



void Mesh::ComputeNormals()
{
  if (!normals) {
    std::cerr << "[W] dgc::Mesh::ComputeNormals: normals not initialized! Initializing now... OK" << std::endl;
    normals = new double[nj*ni*3];
  }

  double p[3]; // point
  double vx[3];  // vector in x-direction
  double vy[3];  // vector in y-direction
  double n[3];   // normal

  for (int i = 0; i < ni; ++i) {
    int j = nj-1;
    int ind = 3*(j + nj*i);
    normals[ind] = 0;
    normals[ind+1] = -1;
    normals[ind+2] = 0;
  }

  for (int j = 0; j < nj; ++j) {
    int i = 0;
    int ind = 3*(j + nj*i);
    normals[ind] = -1;
    normals[ind+1] = 0;
    normals[ind+2] = 0;
  }

  for (int i = 1; i < ni; ++i) {
    for (int j = 0; j < nj-1; ++j) {
      Get(p, i, j);
      Get(vx, i, j+1);
      Get(vy, i-1, j);
      MINUS3(vx, vx, p);
      MINUS3(vy, vy, p);
      CROSS(n, vx, vy);

      double nn = NORM3(n);

      //      cout << "vx= " << vx[0] << " " << vx[1] << " " << vx[2] << endl;
      //      cout << "vy= " << vy[0] << " " << vy[1] << " " << vy[2] << endl;
      //      cout << "n= " << n[0]/nn << " " << n[1]/nn << " " << n[2]/nn << endl;

      int ind = 3*(j + nj*i);

      normals[ind] = n[0]/nn;
      normals[ind+1] = n[1]/nn;
      normals[ind+2] = n[2]/nn;
    }
  }

  //  cout << ni << " " <<  nj << endl;
}

void Mesh::Dilate(double r, bool cube) 
{
  if (!odata) {
    odata = new double[ni*nj];
  }
  memcpy(odata, data, ni*nj*sizeof(double));

  // only cube supported for now
  assert(cube);

  int di = (int)ceil(r/cs);
  if (di <= 0)
    return;

  cout << "Mesh::Dilate: dilating with di=" << di << endl;

  for (int i = di; i < ni - di; ++i) {
    for (int j = di; j < nj - di; ++j) {
      // double z = odata[i*nj + j]; // height at center
      for (int k = -di; k <= di; ++k) {
        for (int l = -di; l <= di; ++l) {
          int ik = i + k;
          int jl = j + l;
          data[ik*nj + jl] = max(data[ik*nj + jl], odata[ik*nj + jl]);
        }        
      }
    }
  }
  ComputeNormals();
  //  memcpy(odata, data, ni*nj*sizeof(double));
}

void Mesh::Convolve(double sigma, bool cn, double thresh)
{
  if (!odata) {
    odata = new double[ni*nj];
    memcpy(odata, data, ni*nj*sizeof(double));
  }

  int a = (int)round(2*sigma/cs);

  for (int i = 0; i < ni; ++i) {
    for (int j = 0; j < nj; ++j) {

      //      int jl = MAX(j-a,0);
      //      int jr = MIN(j+a,nj-1);
      //     int il = MAX(i-a,0);
      //     int ir = MIN(i+a,ni-1);
      
      double z = 0;
      double n = 0;
      
      //      for (int ci = il; ci <= ir; ++ci) {
      //        for (int cj = jl; cj <= jr; ++cj) {
      for (int ci = i-a; ci <= i+a; ++ci) {
        for (int cj = j-a; cj <= j+a; ++cj) {
          
          double &d = odata[cj + nj*ci];
          if (d < thresh)
            continue;
          double r =  (cj-j)*(cj-j) + (ci-i)*(ci-i);
          double g = 1/(sigma*sqrt(2*M_PI))*exp(-r*cs*cs/(2*sigma*sigma));
          n += g;
          if (ci >= 0 && ci < ni && cj >= 0 && cj < nj)
            z += g*d;
        }
      }
      data[j + nj*i] = z/n;
    }
  }
  if (cn)
    ComputeNormals();
}



bool Mesh::Inside(double x, double y, double z) const
{
  if (IsValid(x,y))
    return Get(x,y) > z;
  return false;
}


void Mesh::Set(int i, int j, double z)
{
  //  cout << x << " " << y << endl;
  assert(i>= 0  && i <ni);
  assert(j>= 0  && j <nj);
  data[i*nj + j] = z;
}

void Mesh::Set(double x, double y, double z, double s)
{
  //  cout << x << " " << y << endl;
  if (!IsValid(x, y)) {
    std::cerr << "Warning: Mesh::Set: invalid (x,y)=(" << x << "," << y << ")" << std::endl;
  }
  
  if (s < eps) {
    data[(int)((x - o[0])/cs) + nj*((int)((h - y - o[1])/cs))] = z;
  } else {
    for (double xs = x - s; xs <= x + s; xs += cs)
      for (double ys = y - s; ys <= y + s; ys += cs)
        if (IsValid(xs, ys))
          data[(int)((xs - o[0])/cs) + nj*((int)((h - ys - o[1])/cs))] = z;
  }
}

void Mesh::Clear()
{
  memset(data, 0, ni*nj*sizeof(double));
}

const double* Mesh::GetNormal(double x, double y) const
{
  if (!IsValid(x, y)) {
    return 0;
  }
  int ind = 3*((int)((x - o[0])/cs) + nj*((int)((h - y - o[1])/cs)));
  return normals + ind;
}


double Mesh::GetNormal(double n[3], double x, double y) const
{
  if (!IsValid(x, y)) {
    n[0] = 0;
    n[1] = 0;
    n[2] = 1;
    return 0;
  }
  int ind = 3*((int)((x - o[0])/cs) + nj*((int)((h - y - o[1])/cs)));
  SET3(n, normals+ind);
  
  return o[2] + bilinterp(data, nj, ni, (x - o[0])/cs, (h - y - o[1])/cs);
}

double Mesh::Get(double x, double y) const
{
  //  cout << x << " " << y << endl;
  if (!IsValid(x, y)) {
    //    std::cerr << "[W] dgc::Mesh::Get: invalid (x,y)=(" << x << "," << y << ")" << std::endl;
    return 0;
  }
  return o[2] + bilinterp(data, nj, ni, (x - o[0])/cs, (h - y - o[1])/cs);
}

void Mesh::Get(double *p, int i, int j) const
{
  if (i < 0 || i >= ni ||
      j < 0 || j >= nj) {
    std::cerr << "Warning: Mesh::Get: invalid (i,j)=(" << i << "," << j << ")" << std::endl;
    return;
  }

  p[0] = j*cs + o[0];
  p[1] = h - i*cs + o[1];
  p[2] = data[i*nj + j] + o[2];
}

void Mesh::Scale(double s)
{
  for (int i = 0; i < ni; ++i)
    for (int j = 0; j < nj; ++j)
      data[i*nj + j] *= s;
}


void Mesh::AddBoundary(double h)
{
  for (int i = 0; i < ni; ++i) {
    data[0 + nj*i] = h;
    data[nj-1 + nj*i] = h;
  }
  for (int j = 0; j < nj; ++j) {
    data[j + nj*0] = h;
    data[j + nj*(ni-1)] = h;
  } 
}


void Mesh::Load(const char *fname)
{
  FILE* file = fopen(filename,"rb");
  if(file)
  {
    int size=0;
    if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET))
    {
      printf("Error: Cannot access file to determine size of %s\n", filename);
    } 
    else
    {
      if (size)
      {
        printf("Open STL file of %d bytes\n",size);
        char* memoryBuffer = new char[size+1];
        int actualBytesRead = fread(memoryBuffer,1,size,file);
        if (actualBytesRead!=size)
        {
          printf("Error reading from file %s",filename);
        } 
        else
        {
          int numTriangles = *(int*)&memoryBuffer[80];

          if (numTriangles)
          {
            {
              //perform a sanity check instead of crashing on invalid triangles/STL files
              int expectedBinaryFileSize = numTriangles* 50 + 84;
              if (expectedBinaryFileSize != size)
              {
                return 0;
              }

            }
            btScalar *verts = (btScalar *)malloc(3*3*numTriangles*sizeof(btScalar));//No compression used Can use trees if needed later 3 vertices per triangle, 3 indices per vertex
            int *inds = (int *)malloc(3*numTriangles*sizeof(int));
            int i;

            {
              float *vert_temp = (float *)malloc(3*3*numTriangles*sizeof(float));

#pragma omp parallel for private(i)
              for (i=0;i<numTriangles;i++)
              {
                memcpy(&vert_temp[9*i],&memoryBuffer[96+i*50],36);//9 floats of 4 bytes each (3 loats per vertex)
                inds[3*i+0] = 3*i;
                inds[3*i+1] = 3*i+1;
                inds[3*i+2] = 3*i+2;
                //cout<<"Vertices["<<3*i<<"]: "<<verts[3*i]<<"\t"<<verts[3*i+1]<<"\t"<<verts[3*i+2]<<endl;
                //cout<<"Vertices["<<(3*i+1)<<"]: "<<verts[3*(i+1)]<<"\t"<<verts[3*(i+1)+1]<<"\t"<<verts[3*(i+1)+2]<<endl;
                //cout<<"Vertices["<<(3*i+2)<<"]: "<<verts[3*(i+2)]<<"\t"<<verts[3*(i+2)+1]<<"\t"<<verts[3*(i+2)+2]<<endl;
              }

              //Convert float vertices into  double vertices:
#pragma omp parallel for private(i)
              for(i = 0; i < 9*numTriangles; i++)
              {
                verts[i] = double(vert_temp[i]);//explicit casting 
              }
            }

            //Scale the vertices accordingly
            if((scale - btVector3(1,1,1)).norm()>1e-6)
            {
#pragma omp parallel for private(i)
              for(i = 0;i < 9*numTriangles;i++)
              {
                verts[i] *= scale[i%3];
              }
            }

            int vertStride = 3*sizeof(btScalar); 
            int indexStride = 3*sizeof(int);
            btTriangleIndexVertexArray *m_indexVertexArrays = new btTriangleIndexVertexArray(numTriangles
                ,inds
                ,indexStride
                ,3*numTriangles,verts,vertStride);
            return m_indexVertexArrays;
            //return CreateMeshFromData(verts, inds, numTriangles, 3*numTriangles);
          }
          delete[] memoryBuffer;
        }
      }
      fclose(file);
    }
  }
  return 0;
}

/*{
  int i, size;
  char* chdata;
  fstream fstr;

  fstr.open(fname, std::ios::in);
  
  if (!fstr.good()) {
    std::cerr << "[E]: Mesh:Load - failed to open " << string(fname) << std::endl;
    return;
  }
  
  char line[256];
  
  fstr.getline(line, 256);
  if (line[0] != 'P' || line[1] != '6') {
    cerr << "[E]: Mesh:Load - file " << fname << " should be in P6 binary format" << endl;
    return;
  }
  
  // skip comments
  do {
    fstr.getline(line, 256);
  } while(line[0] == '#');
  
  sscanf(line, "%d %d 255\n", &nj, &ni);
  
  size = nj*ni;
  data = new double[size]; // (double*)calloc(size, sizeof(double));
  normals = new double[size*3];
  chdata = new char[3*size]; // (char*)malloc(size*3);
  
  fstr.read(chdata, size*3);
  //  fread(chdata, sizeof(char), size*3, file);
  for (i = 0; i < size; i++) {
    data[i] = (unsigned char)chdata[3*i]*ds/255.0;
  }
  delete[] chdata;
  //  fclose(file);

  fstr.close();

  w = (nj - 1)*cs;
  h = (ni - 1)*cs;
}
*/
