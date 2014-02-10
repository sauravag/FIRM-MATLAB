#include "math.h"
#include "class_handle.hpp"

#include <Opcode.h>
using namespace Opcode;

#define DEBUG 0

// Copyright (c) 2013 Vipin Vijayan


//struct Mesh {
//  int *faces;
//  float *vertices;
//};
// static void trivertsCallback(udword trix, VertexPointers& triangle, void* data)
// {
//   Mesh *fv = (Mesh*)data;
//   float *v = fv->vertices;
//   int f0 = fv->faces[3*trix];
//   int f1 = fv->faces[3*trix+1];
//   int f2 = fv->faces[3*trix+2];
//   triangle.Vertex[0] = IceMaths::Point(v[3*f0],v[3*f0+1],v[3*f0+2]);
//   triangle.Vertex[1] = IceMaths::Point(v[3*f1],v[3*f1+1],v[3*f1+2]);
//   triangle.Vertex[2] = IceMaths::Point(v[3*f2],v[3*f2+1],v[3*f2+2]);
// }

// void convertfv(mxArray *f, mxArray *v, mxArray **faces, mxArray **vertices) {		
//   mexCallMATLAB(1, vertices, 1, &v, "single");
//   mexCallMATLAB(1, faces, 1, &f, "int32");
//   int *facesptr;
//   facesptr = (int*)mxGetData(faces);
//   int n_f = (int) mxGetN(f);
//   for(int ni=0; ni<3*n_f; ni++) { facesptr[ni]--; } // 1-index -> 0-index
// }

struct Mesh {
  IceMaths::Point *cVertices;
  IceMaths::IndexedTriangle *cIndices;
};
struct Tree {
  Mesh fv;
  Model op;
};

static void trivertsCallback(udword triangleindex, VertexPointers& triangle, void* data)
{
  Mesh *m = (Mesh*)data;
  IceMaths::IndexedTriangle *t = &(m->cIndices[triangleindex]);
  triangle.Vertex[0] = &(m->cVertices[t->mVRef[0]]);
  triangle.Vertex[1] = &(m->cVertices[t->mVRef[1]]);
  triangle.Vertex[2] = &(m->cVertices[t->mVRef[2]]);
}
void convertfv(mxArray *fm, mxArray *vm, Mesh *fv) {
  int n_f = (int) mxGetN(fm);
  int n_v = (int) mxGetN(vm);
  int *f = (int*)mxGetData(fm);
  float *v = (float*)mxGetData(vm);
  int ki;
  
  IceMaths::IndexedTriangle* fc = new IceMaths::IndexedTriangle [n_f];
  for (ki=0; ki < n_f; ki++) {
	fc[ki] = IceMaths::IndexedTriangle(f[ki*3]-1,f[ki*3+1]-1,f[ki*3+2]-1);
  }
  IceMaths::Point *vc = new IceMaths::Point [n_v];
  for (ki=0; ki < n_v; ki++) {
	vc[ki] = IceMaths::Point(v[ki*3],v[ki*3+1],v[ki*3+2]);
  }
  fv->cIndices = fc;
  fv->cVertices = vc;
}

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	
  if (nrhs < 1 || !(mxIsChar(prhs[0]))) {
	mexErrMsgTxt("Bad input.");
  }
	
  int cmdlen = (mxGetM(prhs[0]) * mxGetN(prhs[0])) + 1;
  char *cmd = (char*)mxCalloc(cmdlen, sizeof(char));
  mxGetString(prhs[0], cmd, cmdlen);
	
  float nan = sqrtf(-1.0f);
  float inf = MAX_FLOAT; //1.0f/0.0f;
	
  if (strcmp(cmd, "create") == 0) {
		
	int ix_v = 1; int ix_f = 2;

	if (nrhs < 3
		|| mxGetNumberOfDimensions(prhs[ix_v]) != 2
		|| mxGetNumberOfDimensions(prhs[ix_f]) != 2
		|| mxGetM(prhs[ix_v]) != 3
		|| mxGetM(prhs[ix_f]) != 3)
	  mexErrMsgTxt("Requires vertices (3 x n_v) and faces (3 x n_f).");
		
	int n_v = (int) mxGetN(prhs[ix_v]);
	int n_f = (int) mxGetN(prhs[ix_f]);

	Mesh *fv = new Mesh;
	mxArray *fmat; mxArray *vmat;
	mexCallMATLAB(1, &fmat, 1,(mxArray **)prhs+ix_f, "int32");	
	mexCallMATLAB(1, &vmat, 1,(mxArray **)prhs+ix_v, "single");	
	convertfv(fmat,vmat,fv);

	// Create the tree
	Model *opcode = new Model;
	MeshInterface *cMesh = new MeshInterface;
	cMesh->SetInterfaceType(MESH_TRIANGLE);
	cMesh->SetNbTriangles(n_f);
	cMesh->SetNbVertices(n_v);
	//cMesh->SetCallback(trivertsCallback,(void*)fv);
	cMesh->SetPointers(fv->cIndices,fv->cVertices);
	OPCODECREATE OPCC;
	OPCC.mIMesh = cMesh;
	BuildSettings cBS;
    cBS.mRules = SPLIT_SPLATTER_POINTS | SPLIT_GEOM_CENTER;
	OPCC.mSettings = cBS;
	OPCC.mNoLeaf = true;
	OPCC.mQuantized = false;
	OPCC.mKeepOriginal = false; // For debug
	bool status = opcode->Build(OPCC);

	if (!status) {
	  mexErrMsgTxt("Error when making tree.");
	}

	plhs[0] = convertPtr2Mat<Model>(opcode);
	//plhs[1] = convertPtr2Mat<MeshInterface>(cMesh);
	//plhs[1] = convertPtr2Mat<IceMaths::IndexedTriangle>(fv->cIndices);
	//plhs[2] = convertPtr2Mat<IceMaths::Point>(fv->cVertices);
  }
  else if (strcmp(cmd, "intersect") == 0) {
		
	int ix_t = 1; int ix_c1 = 2; int ix_c2 = 3;
		
	if (nrhs < 4
	    || mxGetNumberOfDimensions(prhs[ix_c1]) != 2
		|| mxGetNumberOfDimensions(prhs[ix_c2]) != 2
		|| mxGetM(prhs[ix_c1]) != 3
		|| mxGetM(prhs[ix_c2]) != 3
		|| mxGetN(prhs[ix_c1]) != mxGetN(prhs[ix_c2]))
	  mexErrMsgTxt("Req. aabb tree handle, ray start (3 x n_c) and ray dir. (3 x n_c).");			
		
	Model *opcode = convertMat2Ptr<Model>(prhs[ix_t]);
	//MeshInterface *cMesh = convertMat2Ptr<MeshInterface>(prhs[ix_tf]);
	//opcode->SetMeshInterface(cMesh);
	//IceMaths::IndexedTriangle *fvf = convertMat2Ptr<IceMaths::IndexedTriangle>(prhs[ix_tf]);
	//IceMaths::Point *fvv = convertMat2Ptr<IceMaths::Point>(prhs[ix_tv]);
	RayCollider RC;
	RC.SetFirstContact(false);
	RC.SetClosestHit(true);
	RC.SetCulling(false);
	RC.SetMaxDist(inf);
	CollisionFaces CF;
	RC.SetDestination(&CF);

	int n_c = (int) mxGetN(prhs[ix_c1]);
	int ix_hit=0,ix_dist=1,ix_tridx=2,ix_bary=3;

	mxArray *rayorig;
	mxArray *raydir;	
	mexCallMATLAB(1, &rayorig, 1,(mxArray **)prhs+ix_c1, "single");	
	mexCallMATLAB(1, &raydir, 1,(mxArray **)prhs+ix_c2, "single");	
		
	// Create an output array of indices
	unsigned char *hitptr = 0;
	plhs[ix_hit] = mxCreateNumericMatrix(n_c,1,mxLOGICAL_CLASS,mxREAL);
	hitptr = (unsigned char*) mxGetData(plhs[ix_hit]);
		
	float *distptr = 0;
	if (nlhs > ix_dist) {
	  plhs[ix_dist] = mxCreateNumericMatrix(n_c, 1, mxSINGLE_CLASS, mxREAL);
	  distptr = (float*) mxGetData(plhs[ix_dist]);
	}
		
	int*tridxptr = 0;
	if (nlhs > ix_tridx) {
	  plhs[ix_tridx] = mxCreateNumericMatrix(n_c, 1, mxINT32_CLASS, mxREAL);
	  tridxptr = (int *) mxGetData(plhs[ix_tridx]);
	}

	float *baryptr = 0;
	if (nlhs > ix_bary) {
	  plhs[ix_bary] = mxCreateNumericMatrix(2, n_c, mxSINGLE_CLASS, mxREAL);
	  baryptr = (float *) mxGetData(plhs[ix_bary]);
	}

	float *rayo = (float*)mxGetData(rayorig);
	float *rayd = (float*)mxGetData(raydir);
	bool hit,status;
	int i;
	IceMaths::Point cStart, cDir;
	IceMaths::Ray cRay;
	
	for (i=0; i<n_c; i++) {
	  IceMaths::Point cStart = IceMaths::Point(rayo[3*i],rayo[3*i+1],rayo[3*i+2]);
	  IceMaths::Point cDir = IceMaths::Point(rayd[3*i],rayd[3*i+1],rayd[3*i+2]);
	  //cDir.Normalize();
	  IceMaths::Ray cRay = IceMaths::Ray(cStart,cDir);
	  static udword Cache;
	  status = RC.Collide(cRay, *opcode, NULL, &Cache);
	  if (!status) mexErrMsgTxt("Error when hitting.");
	  hit = RC.GetContactStatus();
	  
	  hitptr[i] = (unsigned char)hit;
	  //	printf("hit %d hitd %f\n",hit,hitDistance);

	  const CollisionFace* colFaces;
	  if (nlhs > ix_dist) {
		colFaces = CF.GetFaces();
		//	  printf("hitD: %f\n", hitDistance);
		distptr[i] = hit ? colFaces[0].mDistance : nan;
	  }
	  if (nlhs > ix_tridx) {
		tridxptr[i] = hit ? (colFaces[0].mFaceID+1) : 0; // 1-based
	  }
	  if (nlhs > ix_bary) {
		baryptr[2*i] = hit ? colFaces[0].mU : nan;
		baryptr[2*i+1] = hit ? colFaces[0].mV : nan;
		//printf("hitL: %f %f %f\n",
		//hitLocation[0],hitLocation[1],hitLocation[2]);
	  }
	}
  }
  else if (strcmp(cmd, "update") == 0) {
		
	int ix_t = 1; int ix_v = 2;
		
	if (nrhs < 3 || mxGetM(prhs[ix_v]) != 3)
	  mexErrMsgTxt("Req. aabb tree handle, vertices (3 x n_v).");			
		
	Model *opcode = convertMat2Ptr<Model>(prhs[ix_t]);

	int n_v = (int) mxGetN(prhs[ix_v]);
	if (n_v != opcode->GetMeshInterface()->GetNbVertices()) {
		mexErrMsgTxt("Input vertices need to match current mesh.");
	}

	mxArray *vertices;
	mexCallMATLAB(1, &vertices, 1,(mxArray **)prhs+ix_v, "single");
	float *v = (float*)mxGetData(vertices);
	IceMaths::Point* vc = (IceMaths::Point*)opcode->GetMeshInterface()->GetVerts();
	for (int ki=0; ki<n_v; ki++) {
		vc[ki] = IceMaths::Point(v[ki*3],v[ki*3+1],v[ki*3+2]);
	}
	opcode->Refit();
  }
  else if (strcmp(cmd, "delete") == 0) {
	int ix_t = 1;
	if (nrhs < 2) mexErrMsgTxt("Requires aabb tree.");
	destroyObject<Model>(prhs[ix_t]);
  }
  else {
	mexErrMsgTxt("Command not recognized.");
  }
}

