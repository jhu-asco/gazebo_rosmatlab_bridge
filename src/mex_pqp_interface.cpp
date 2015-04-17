/* This is an interface to load PQP meshes into MATLAB and perform collision checking between two meshes
 * #TODO Add lots of checks on whether meshes exist, PQPModel exists, arguments given correctly etc
*/
/* system header */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* MEX header */
#include <mex.h> 
//#include "matrix.h"
#include <gazebo_rosmatlab_bridge/mex_utils.h>

#include "PQP/PQP.h"

#include <gazebo_rosmatlab_bridge/pqpmesh.h>

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
      if(nlhs != 1 && nrhs != 4)
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, mesh_name1, mesh_name2} and output: distance between meshes");
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

      // subtract safety distance
      //d = MAX(0, d - sd);//#TODO


      //  if (dem.Inside(dres.P1()[0], dres.P1()[1], dres.P1()[2]))

      //#TODO Add normal of the closest point

      plhs[0] = mxCreateDoubleScalar(mesh_distance);
    }
  }
  else
  {
    mexPrintf("\n No rhs arguments\n");
    return;
  }
}
