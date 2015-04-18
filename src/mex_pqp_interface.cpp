/* This is an interface to load PQP meshes into MATLAB and perform collision checking between two meshes
 * #TODO Add lots of checks on whether meshes exist, PQPModel exists, arguments given correctly etc
*/
/* system header */
//#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gazebo_rosmatlab_bridge/pqpmesh.h>

/* MEX header */
#include <mex.h> 
//#include "matrix.h"
#include <gazebo_rosmatlab_bridge/mex_utils.h>

#include "PQP/PQP.h"


#include <vector>
#include <map>

//Useful Links:
//http://www.mathworks.com/matlabcentral/answers/uploaded_files/1750/simplefunction.cpp

using namespace std;

/** This class is used for storing data that needs to be saved even after returning the mex function.
 *   This essentially contains the mesh files that are loaded everytime 
 */
class Data_struct
{
  public:
    std::map<std::string, PqpMesh* > meshes; ///< Mesh map storing all the meshes by their unique id
    PQP_DistanceResult dres; ///< Distance result from PQP
};

/* MEX entry function */
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  char cmd[64];
  if(nrhs >= 1)
  {
    if(mxGetString(prhs[0], cmd, sizeof(cmd)))
      mexErrMsgTxt("First input should be a command string less than 64 characters long.");

    if (!strcmp("new", cmd)) 
    { 
      Data_struct *storage = new Data_struct();//Create a storage space for meshes
      if(nlhs != 1)
      {
        mexErrMsgTxt("There should be only one output for storing data in new");
        return;
      }
      plhs[0] = convertPtr2Mat<Data_struct>(storage); 

      return;
    }
    else if(!strcmp("delete",cmd))
    {
      if(nrhs != 2)
      {
        mexErrMsgTxt("To delete need to pass the storage data");
        return;
      }
      Data_struct *storage = convertMat2Ptr<Data_struct>(prhs[1]);
      destroyObject<Data_struct>(prhs[1]);
      return;
    }
    else if(!strcmp("loadmesh",cmd))
    {
      if(nlhs != 0 && nrhs < 4)
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, mesh_name, mesh_path/collisionradius, mesh_state} and output none; mesh_state = [7x1] position and orientation; mesh_path is optional");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
      char modelname[100];
      mxGetString(prhs[2], modelname, sizeof(modelname));
      printf("Modelname: %s\n",modelname);

      if(mxIsChar(prhs[3]))
      {
        char mesh_path[1000];
        mxGetString(prhs[3], mesh_path, sizeof(mesh_path));
        printf("Mesh Path: %s",mesh_path);
        //TODO Add Mesh scale
        PqpMesh *newmesh = new PqpMesh(mesh_path);
        d->meshes.insert(std::pair<std::string, PqpMesh* >(std::string(modelname), newmesh));//We are not replacing anything
      }
      else
      {
        //If not a string Try reading the collision radius:
        double collision_radius = mxGetScalar(prhs[3]);
        PqpMesh *newmesh = new PqpMesh(collision_radius);
        d->meshes.insert(std::pair<std::string, PqpMesh* >(std::string(modelname), newmesh));//We are not replacing anything
      }

      if(nrhs == 5)
      {
        //Set mesh position and orientation of the mesh:
        double *modelstate = mxGetPr(prhs[4]);

        PqpMesh *currentmesh = d->meshes[std::string(modelname)];
        //Position
        currentmesh->pt[0] = modelstate[0];
        currentmesh->pt[1] = modelstate[1];
        currentmesh->pt[2] = modelstate[2];
        //Orientation from quat2mat
        {
          double qw = modelstate[3], qx = modelstate[4], qy = modelstate[5], qz = modelstate[6];

          currentmesh->pR[0][0] = 1 - 2*qy*qy - 2*qz*qz; currentmesh->pR[0][1] = 2*qx*qy - 2*qz*qw;   currentmesh->pR[0][2] = 2*qx*qz + 2*qy*qw;
          currentmesh->pR[1][0] = 2*qx*qy + 2*qz*qw;   currentmesh->pR[1][1] = 1 - 2*qx*qx - 2*qz*qz; currentmesh->pR[1][2] = 2*qy*qz - 2*qx*qw;
          currentmesh->pR[2][0] = 2*qx*qz - 2*qy*qw;   currentmesh->pR[2][1] = 2*qy*qz + 2*qx*qw;   currentmesh->pR[2][2] =  1 - 2*qx*qx - 2*qy*qy;
        }
      }
    }
    else if(!strcmp("setmeshstate",cmd))
    {
      if(nlhs != 1 && nrhs != 4)
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, mesh_name1, mesh_state} and output: none");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);

      char mesh_name[64];
      int status = mxGetString(prhs[2], mesh_name, sizeof(mesh_name));
      assert(status == 0);//Make sure read is successful
      PqpMesh *currentmesh = d->meshes[std::string(mesh_name)];//Get first mesh

      //Set mesh position and orientation of the mesh:
      double *modelstate = mxGetPr(prhs[3]);

      //Position
      currentmesh->pt[0] = modelstate[0];
      currentmesh->pt[1] = modelstate[1];
      currentmesh->pt[2] = modelstate[2];
      //Orientation from quat2mat
      {
        double qw = modelstate[3], qx = modelstate[4], qy = modelstate[5], qz = modelstate[6];

        currentmesh->pR[0][0] = 1 - 2*qy*qy - 2*qz*qz; currentmesh->pR[0][1] = 2*qx*qy - 2*qz*qw;   currentmesh->pR[0][2] = 2*qx*qz + 2*qy*qw;
        currentmesh->pR[1][0] = 2*qx*qy + 2*qz*qw;   currentmesh->pR[1][1] = 1 - 2*qx*qx - 2*qz*qz; currentmesh->pR[1][2] = 2*qy*qz - 2*qx*qw;
        currentmesh->pR[2][0] = 2*qx*qz - 2*qy*qw;   currentmesh->pR[2][1] = 2*qy*qz + 2*qx*qw;   currentmesh->pR[2][2] =  1 - 2*qx*qx - 2*qy*qy;
      }
    }
    else if(!strcmp("computedistance",cmd))
    {
      if(nlhs < 1 && nrhs != 4)
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, mesh_name1, mesh_name2} and output: distance between meshes; Optional output: closest points in mesh2 frame");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);

      //Get both mesh names

      char mesh_name[64];
      int status = mxGetString(prhs[2], mesh_name, sizeof(mesh_name));
      assert(status == 0);//Make sure read is successful
      PqpMesh *mesh1 = d->meshes[std::string(mesh_name)];//Get first mesh

      status = mxGetString(prhs[3], mesh_name, sizeof(mesh_name));
      assert(status == 0);//Make sure read is successful
      PqpMesh *mesh2 = d->meshes[std::string(mesh_name)];//Get second mesh

      //Compute the distance between the meshes:
      int res = PQP_Distance(&(d->dres), mesh1->pR, mesh1->pt, mesh1->pm, mesh2->pR, mesh2->pt, mesh2->pm, 0.0, 0.0);

      if (res != PQP_OK)
        cerr << "Warning: PqpDem:Distance: res=" << res << endl;

      assert(res == PQP_OK);

      double mesh_distance = (d->dres).Distance();
      assert(mesh_distance>=0);

      //Check if mesh1 is inside mesh2:

      //First find normal corresponding to mesh2
      Tri *t2 = (mesh2->pm)->last_tri;//Last triangle from PQP Model
      tf::Vector3 edge1((t2->p2[0] - t2->p1[0]),(t2->p2[1] - t2->p1[1]),(t2->p2[2] - t2->p1[2]));
      tf::Vector3 edge2((t2->p3[0] - t2->p1[0]),(t2->p3[1] - t2->p1[1]),(t2->p3[2] - t2->p1[2]));
      tf::Vector3 normal = (edge1.cross(edge2)).normalized();

      //Get the vector pointing one mesh to other in the frame of mesh 2:
      const PQP_REAL *point1 = (d->dres).P1();
      const PQP_REAL *point2 = (d->dres).P2();
      //printf("P1: %f,%f,%f\n",point1[0], point1[1], point1[2]);
      //printf("P2: %f,%f,%f\n",point2[0], point2[1], point2[2]);

      tf::Vector3 p1(point1[0],point1[1],point1[2]);
      tf::Vector3 p2(point2[0],point2[1],point2[2]);
      tf::Transform mesh_state12 = (mesh2->GetState()).inverse()*(mesh1->GetState());//Converts points in mesh1 frame to mesh2 frame

      
      //Transform p1 into mesh2 frame and substract it from p2:
      tf::Vector3 p1_2 = (mesh_state12*p1);
      tf::Vector3 p12 = p2 - p1_2;
      p12.normalize();//Normalize the vector between the meshes

      //Compute Inner product:
      double inner_product = p12.dot(normal);
      //printf("Inner product: %f\n",inner_product);
      if(inner_product > 0)//Can add tolerance here #TODO
        mesh_distance = -mesh_distance;

      // subtract safety distance
      //d = MAX(0, d - sd);//#TODO

      plhs[0] = mxCreateDoubleScalar(mesh_distance);
      if(nlhs >= 2)
      {
        plhs[1] = mxCreateDoubleMatrix(3,2,mxREAL);
        double *pointer = mxGetPr(plhs[1]);
        pointer[0] = p1_2.x(); pointer[1] = p1_2.y(); pointer[2] = p1_2.z();
        pointer[3] = p2.x(); pointer[4] = p2.y(); pointer[5] = p2.z();
      }
      if(nlhs >=3)
      {
        plhs[2] = mxCreateDoubleMatrix(3,3,mxREAL);
        double *pointer = mxGetPr(plhs[2]);
        pointer[0] = t2->p1[0]; pointer[1] = t2->p1[1]; pointer[2] = t2->p1[2];
        pointer[3] = t2->p2[0]; pointer[4] = t2->p2[1]; pointer[5] = t2->p2[2];
        pointer[6] = t2->p3[0]; pointer[7] = t2->p3[1]; pointer[8] = t2->p3[2];
      }
    }
  }
  else
  {
    mexPrintf("\n No rhs arguments\n");
    return;
  }
}
