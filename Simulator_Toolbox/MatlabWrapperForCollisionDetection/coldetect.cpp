#include "mex.h"
#include "VCollide.H"

void addTris(double *tris, int n, VCollide &vc) {
    int id;
    double v1[3], v2[3], v3[3];
    vc.NewObject(&id);
    for (int j(0); j < n; j++) {
        v1[0] = tris[j + 0 * n];
        v1[1] = tris[j + 1 * n];
        v1[2] = tris[j + 2 * n];

        v2[0] = tris[j + 3 * n];
        v2[1] = tris[j + 4 * n];
        v2[2] = tris[j + 5 * n];

        v3[0] = tris[j + 6 * n];
        v3[1] = tris[j + 7 * n];
        v3[2] = tris[j + 8 * n];    
        vc.AddTri(v1, v2, v3);
    }
    vc.EndObject();
}

void update(double *tr, int t, int ntrans, double vc_trans[][4]) {
    vc_trans[0][0] = tr[t + 3 * ntrans];
    vc_trans[1][0] = tr[t + 6 * ntrans];
    vc_trans[2][0] = tr[t + 9 * ntrans];
    vc_trans[3][0] = 0;
    vc_trans[0][1] = tr[t + 4 * ntrans];
    vc_trans[1][1] = tr[t + 7 * ntrans];
    vc_trans[2][1] = tr[t + 10 * ntrans];
    vc_trans[3][1] = 0;
    vc_trans[0][2] = tr[t + 5 * ntrans];
    vc_trans[1][2] = tr[t + 8 * ntrans];
    vc_trans[2][2] = tr[t + 11 * ntrans];
    vc_trans[3][2] = 0;
    vc_trans[0][3] = tr[t + 0 * ntrans];
    vc_trans[1][3] = tr[t + 1 * ntrans];
    vc_trans[2][3] = tr[t + 2 * ntrans];
    vc_trans[3][3] = 1;
}

int coldetect(int ntri1, int ntri2, int ntrans1, int ntrans2, double *tri1, double *tri2, double *trans1, double *trans2) {
    double vc_trans[4][4];
    VCReport report;
    VCollide vc;
    addTris(tri1, ntri1, vc);
    addTris(tri2, ntri2, vc);
    // Iterate through transformations
    for (int t(0); t < ntrans1; t++) {
        update(trans1, t, ntrans1, vc_trans);
        vc.UpdateTrans(0, vc_trans);
        update(trans2, t, ntrans2, vc_trans);
        vc.UpdateTrans(1, vc_trans);
        vc.Collide(&report);
        if (report.numObjPairs() > 0) {
            return t + 1;
        }
    }
    return 0;
}


void mexFunction(int nlhs, mxArray *plhs[], 
    int nrhs, const mxArray *prhs[])
{
    int ntri1, ntri2, ntrans1, ntrans2;
    double *tri1, *tri2, *trans1, *trans2;
    if (nrhs != 4) mexErrMsgTxt("Expecting four input arguments.");
    if (nlhs != 1) mexErrMsgTxt("Expecting one output argument.");
    if (mxGetN(prhs[0]) != 9) mexErrMsgTxt("First argument must have nine columns.");
    if (mxGetN(prhs[1]) != 9) mexErrMsgTxt("Second argument must have nine columns.");
    if (mxGetN(prhs[2]) != 12) mexErrMsgTxt("Third argument must have twelve columns.");
    if (mxGetN(prhs[3]) != 12) mexErrMsgTxt("Fourth argument must have twelve columns.");

    ntri1 = mxGetM(prhs[0]);
    ntri2 = mxGetM(prhs[1]);
    ntrans1 = mxGetM(prhs[2]);
    ntrans2 = mxGetM(prhs[3]);
    if (ntrans1 != ntrans2) mexErrMsgTxt("Third and fourth arguments must have same number of rows.");
    tri1 = (double *) mxGetPr(prhs[0]); 
    tri2 = (double *) mxGetPr(prhs[1]); 
    trans1 = (double *) mxGetPr(prhs[2]); 
    trans2 = (double *) mxGetPr(prhs[3]); 

    plhs[0] = mxCreateDoubleScalar(coldetect(ntri1, ntri2, ntrans1, ntrans2, tri1, tri2, trans1, trans2));
   
}
