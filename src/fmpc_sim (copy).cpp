#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "fub_mpc_planner/blas.h"
#include "fub_mpc_planner/lapack.h"

const int ione = 1;
const int itwo = 2;
const int ithree = 3;
const int iseven = 7;
const double fone = 1;
const double ftwo = 2;
const double fzero = 0;
const double fmone = -1;
int quiet = 0;

void printmat(double *A, int m, int n);
void fmpcsolve(double *A, double *B, double *At, double *Bt, double *eyen,
        double *eyem, double *Q, double *R, double *Qf, double *zmax, double *zmin,
        double *x, double *z, int T, int n, int m, int nz, int niters, double kappa);

void gfgphp(double *Q, double *R, double *Qf, double *zmax, double *zmin, double *z,
        int T, int n, int m, int nz, double *gf, double *gp, double *hp);

void rdrp(double *A, double *B, double *Q, double *R, double *Qf, double *z, double *nu,
        double *gf, double *gp, double *b, int T, int n, int m, int nz,
        double kappa, double *rd, double *rp, double *Ctnu);

void resdresp(double *rd, double *rp, int T, int n, int nz, double *resd,
        double *resp, double *res);

void dnudz(double *A, double *B, double *At, double *Bt, double *eyen,
        double *eyem, double *Q, double *R, double *Qf, double *hp, double *rd,
        double *rp, int T, int n, int m, int nz, double kappa, double *dnu, double *dz);



int main(int argc, char **argv)
{
    /* inputs*/
 /* problem setup */
    double delta_t=0.1;
    /* problem setup */
    int i, j, m, n, nz, T, niters, nsteps, info;
    double kappa;
    double *dptr, *dptr1, *dptr2, *y1, *y2;
    double *A, *B, *At, *Bt, *Q, *R, *Qf, *xmax, *xmin, *umax, *umin, *x, *u;
    double *zmax, *zmin, *Xhist, *Uhist, *z, *eyen, *eyem, *x0, *w;
    double *K, *Rtilde, *Rlower, *tempnm, *xterm, *uterm;
    double *umaxp, *uminp, *xmaxp, *xminp, *zmaxp, *zminp;
    double *X0, *U0;
    double *telapsed;
    clock_t t1, t2;

    n=15;
    m=2;
    T=10;
    nsteps = 40;
/*******************system******************************************************
 * X(k+1)=AX+BU+w
********************************************************************************/
    double A_[n*n];
    for (i = 0; i < n*n; i++)
    {
        A_[i]=0;
    }

    for (i = 0; i < n; i++)
    {
        A_[i*n+i]=1;
    }
    i=1;
    while (i < n)
    {
        A_[i*n+i+1]=delta_t;
        i=i+2;
    }
/********************************************************************/
    double B_[n*m];
    for (i = 0; i < n*m; i++)
    {
        B_[i]=0;
    }
    B_[0]=1;
    B_[n+1]=1;
    for (i = 0; i < ((n-3)/4); i++)
    {
        B_[i*4+3]=1;
        B_[n+i*4+5]=1;
    }
 /********************************************************************/
    double w_[n*nsteps];
    for (i = 0; i < n*nsteps; i++)
    {
        w_[i]=0;
    }
 /*******************cost function******************************************************
     * J=XQX+URU
 ********************************************************************************/
    double Q_[n*n];
    for (i = 0; i < n*n; i++)
    {
        Q_[i]=0;
    }
    Q_[2*n+2]=1;
 /********************************************************************/
    double R_[m*m];
    for (i = 0; i < m*m; i++)
    {
        R_[i]=0;
    }
    for (i = 0; i < m; i++)
    {
        R_[i*m+i]=1;
    }
    /*******************state contrain******************************************************
        * xmin < X < xmax
    ********************************************************************************/
    double xmin_[n*T];
    for (i = 0; i < T; i++)
    {
         xmin_[0+i*n]=0;
         xmin_[1+i*n]=-20;
         xmin_[2+i*n]=-4;
         for (j= 0; j < ((n-3)/4); j++)
         {
             xmin_[3+j*4+i*n]=-100;
             xmin_[4+j*4+i*n]=-30;
             xmin_[5+j*4+i*n]=-30;
             xmin_[6+j*4+i*n]=-10;
         }
    };
    double xmax_[n*T];
    for (i = 0; i < T; i++)
    {
         xmax_[0+i*n]=120;
         xmax_[1+i*n]=20;
         xmax_[2+i*n]=8;
         for (j= 0; j < ((n-3)/4); j++)
         {
             xmax_[3+j*4+i*n]=300;
             xmax_[4+j*4+i*n]=100;
             xmax_[5+j*4+i*n]=100;
             xmax_[6+j*4+i*n]=40;
         }
    }

    /*******************input contrain******************************************************
        * umin < U < umax
    ********************************************************************************/
    double umin_[2] = {-0.5,-0.2};//m/s
    double umax_[2] = {0.1,0.2};//m/s
    /*******************initlization******************************************************
        * umin < U < umax
    ********************************************************************************/
    double X0_[n*n];
    double U0_[m*n];
    double x0_[11]={100,0,4,-10,100,0,0,10,-100,0,-4};
    double X_[n*nsteps];
    double U_[m*nsteps];
    double telapsed_;

//ts = 0.5;


    A= &A_[0];
    B =&B_[0];
    Q =&Q_[0];
    R =&R_[0];
    Qf=Q;
    xmin = &xmin_[0];
    xmax = &xmax_[0];
    umin = &umin_[0];
    umax = &umax_[0];


    kappa = 0.01;
    niters = 5;
    quiet = false;

    X0=&X0_[0];
    U0=&U0_[0];
    x0=&x0_[0];
    w=&w_[0];
    nz = T*(n+m);


    Xhist=&X_[0];
    Uhist=&U_[0];
    telapsed=&telapsed_;

    At = (double *)malloc(sizeof(double)*n*n);
    Bt = (double *)malloc(sizeof(double)*n*m);
    eyen = (double *)malloc(sizeof(double)*n*n);
    eyem = (double *)malloc(sizeof(double)*m*m);
    z = (double *)malloc(sizeof(double)*nz);
    x = (double *)malloc(sizeof(double)*n);
    u = (double *)malloc(sizeof(double)*m);
    y1 = (double *)malloc(sizeof(double)*n);
    y2 = (double *)malloc(sizeof(double)*n);
    zmax = (double *)malloc(sizeof(double)*nz);
    zmin = (double *)malloc(sizeof(double)*nz);
    K = (double *)malloc(sizeof(double)*n*m);
    Rtilde = (double *)malloc(sizeof(double)*m*m);
    Rlower = (double *)malloc(sizeof(double)*m*m);
    tempnm = (double *)malloc(sizeof(double)*n*m);
    xterm = (double *)malloc(sizeof(double)*n);
    uterm = (double *)malloc(sizeof(double)*m);
    umaxp = (double *)malloc(sizeof(double)*m);
    uminp = (double *)malloc(sizeof(double)*m);
    xmaxp = (double *)malloc(sizeof(double)*n);
    xminp = (double *)malloc(sizeof(double)*n);
    zmaxp = (double *)malloc(sizeof(double)*nz);
    zminp = (double *)malloc(sizeof(double)*nz);

    /* eyen, eyem */
    for (i = 0; i < n*n; i++) eyen[i] = 0;
    dptr = eyen;
    for (i = 0; i < n; i++)
    {
        *dptr = 1;
        dptr = dptr+n+1;
    }

    for (i = 0; i < m*m; i++) eyem[i] = 0;
    dptr = eyem;
    for (i = 0; i < m; i++)
    {
        *(dptr+i*m+i) = 1;
    }

    for (i = 0; i < n; i++) x[i] = x0[i];
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr = *(U0+i*m+j);
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr = *(X0+i*n+j);
            dptr++;
        }
    }

    /* At, Bt */
    F77_CALL(dgemm)("t","n",&n,&n,&n,&fone,A,&n,eyen,&n,&fzero,At,&n);
    F77_CALL(dgemm)("n","t",&m,&n,&m,&fone,eyem,&m,B,&n,&fzero,Bt,&m);

    /* zmax, zmin */
    dptr1 = zmax;
    dptr2 = zmin;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr1 = *(umax+j);
            *dptr2 = *(umin+j);
            dptr1++; dptr2++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr1 = *(xmax+j+i*n);
            *dptr2 = *(xmin+j+i*n);
            dptr1++; dptr2++;
        }
    }

    /* zmaxp, zminp */
    for (i = 0; i < nz; i++) zminp[i] = zmin[i] + 0.01*(zmax[i]-zmin[i]);
    for (i = 0; i < nz; i++) zmaxp[i] = zmax[i] - 0.01*(zmax[i]-zmin[i]);

    /* project z */
    for (i = 0; i < nz; i++) z[i] = z[i] > zmaxp[i] ? zmaxp[i] : z[i];
    for (i = 0; i < nz; i++) z[i] = z[i] < zminp[i] ? zminp[i] : z[i];

    /* Rtilde */
    for (i = 0; i < m*m; i++) Rtilde[i] = R[i];
    F77_CALL(dgemm)("n","n",&n,&m,&n,&fone,Qf,&n,B,&n,&fzero,tempnm,&n);
    F77_CALL(dgemm)("t","n",&m,&m,&n,&fone,B,&n,tempnm,&n,&fone,Rtilde,&m);

    /* K */
    F77_CALL(dgemm)("t","n",&m,&n,&n,&fone,B,&n,Qf,&n,&fzero,tempnm,&m);
    F77_CALL(dgemm)("n","n",&m,&n,&n,&fone,tempnm,&m,A,&n,&fzero,K,&m);
    for (i = 0; i < m*m; i++) Rlower[i] = Rtilde[i];
    F77_CALL(dposv)("l",&m,&n,Rlower,&m,K,&m,&info);
    for (i = 0; i < n*m; i++) K[i] = -K[i];

    /* uminp, umaxp, xminp, xmaxp */
    for (i = 0; i < m; i++) uminp[i] = umin[i] + 0.01*(umax[i]-umin[i]);
    for (i = 0; i < m; i++) umaxp[i] = umax[i] - 0.01*(umax[i]-umin[i]);
    for (i = 0; i < n; i++) xminp[i] = xmin[i] + 0.01*(xmax[i]-xmin[i]);
    for (i = 0; i < n; i++) xmaxp[i] = xmax[i] - 0.01*(xmax[i]-xmin[i]);

    t1 = clock();
    for (i = 0; i < nsteps; i++)
    {
        fmpcsolve(A,B,At,Bt,eyen,eyem,Q,R,Qf,zmax,zmin,x,z,T,n,m,nz,niters,kappa);

        /* save x and u to Xhist and Uhist */
        dptr = Xhist+i*n; dptr1 = x;
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = Uhist+i*m; dptr1 = u; dptr2 = z;
        for (j = 0; j < m; j++)
        {
            *dptr = *dptr2;
            *dptr1 = *dptr2;
            dptr++; dptr1++; dptr2++;
        }

        /* compute x = A*x + B*u + w */
        F77_CALL(dgemv)("n",&n,&n,&fone,A,&n,x,&ione,&fzero,y1,&ione);
        F77_CALL(dgemv)("n",&n,&m,&fone,B,&n,u,&ione,&fzero,y2,&ione);
        F77_CALL(daxpy)(&n,&fone,y2,&ione,y1,&ione);
        F77_CALL(daxpy)(&n,&fone,w+n*i,&ione,y1,&ione);
        dptr = x; dptr1 = y1;
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }

        /* shift z for warm start and compute terminal controls*/
        dptr = z;
        for (j = 0; j < nz-n-m; j++)
        {
            *dptr = *(dptr+n+m);
            dptr++;
        }
        F77_CALL(dgemv)("n",&m,&n,&fone,K,&m,z+nz-2*n-m,&ione,&fzero,uterm,&ione);
        F77_CALL(dgemv)("n",&n,&n,&fone,A,&n,z+nz-2*n-m,&ione,&fzero,xterm,&ione);
        F77_CALL(dgemv)("n",&n,&m,&fone,B,&n,u,&ione,&fzero,y2,&ione);
        F77_CALL(daxpy)(&n,&fone,y2,&ione,xterm,&ione);
        for (j = 0; j < m; j++) uterm[j] = uterm[j] > umaxp[j] ? umaxp[j] : uterm[j];
        for (j = 0; j < m; j++) uterm[j] = uterm[j] < uminp[j] ? uminp[j] : uterm[j];
        for (j = 0; j < n; j++) xterm[j] = xterm[j] > xmaxp[j] ? xmaxp[j] : xterm[j];
        for (j = 0; j < n; j++) xterm[j] = xterm[j] < xminp[j] ? xminp[j] : xterm[j];

        for (j = 0; j < m; j++) z[nz-n-m+j] = uterm[j];
        for (j = 0; j < n; j++) z[nz-n+j] = xterm[j];
    }

    t2 = clock();
    *telapsed = (double)(t2-t1)/(CLOCKS_PER_SEC);

    free(At); free(Bt); free(eyen); free(eyem);
    free(z); free(x); free(zmax); free(zmin); free(u); free(y1); free(y2);
    free(K); free(Rtilde); free(Rlower); free(tempnm); free(xterm);
    free(uterm); free(umaxp); free(uminp); free(xmaxp); free(xminp);
    free(zmaxp); free(zminp);
    printmat(Xhist,n,nsteps);
    printf("\n");
    printf("\n");
    printmat(Uhist,m,nsteps);
    return 1;
}

void printmat(double *A, int n, int m)
{
    double *dptr;
    int j, i;
    dptr = A;
    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            printf("%5.4f\t", *(dptr+i+j*n));
        }
        printf("\n");
    }
    return;
}

void fmpcsolve(double *A, double *B, double *At, double *Bt, double *eyen,
         double *eyem, double *Q, double *R, double *Qf, double *zmax, double *zmin,
         double *x, double *z0, int T, int n, int m, int nz, int niters, double kappa)
{
    int maxiter = niters;
    int iiter, i, cont;
    double alpha = 0.01;
    double beta = 0.9;
    double tol = 0.1;
    double s;
    double resd, resp, res, newresd, newresp, newres;
    double *b, *z, *nu, *Ctnu;
    double *dnu, *dz;
    double *gf, *gp, *hp, *newgf, *newgp, *newhp;
    double *rd, *rp, *newrd, *newrp;
    double *dptr, *dptr1, *dptr2, *dptr3;
    double *newnu, *newz, *newCtnu;

    /* memory allocation */
    b = (double *)malloc(sizeof(double)*T*n);
    dnu = (double *)malloc(sizeof(double)*T*n);
    dz = (double *)malloc(sizeof(double)*nz);
    nu = (double *)malloc(sizeof(double)*T*n);
    Ctnu = (double *)malloc(sizeof(double)*nz);
    z = (double *)malloc(sizeof(double)*nz);
    gp = (double *)malloc(sizeof(double)*nz);
    hp = (double *)malloc(sizeof(double)*nz);
    gf = (double *)malloc(sizeof(double)*nz);
    rp = (double *)malloc(sizeof(double)*T*n);
    rd = (double *)malloc(sizeof(double)*nz);
    newnu = (double *)malloc(sizeof(double)*T*n);
    newz = (double *)malloc(sizeof(double)*nz);
    newCtnu = (double *)malloc(sizeof(double)*nz);
    newgf = (double *)malloc(sizeof(double)*nz);
    newhp = (double *)malloc(sizeof(double)*nz);
    newgp = (double *)malloc(sizeof(double)*nz);
    newrp = (double *)malloc(sizeof(double)*T*n);
    newrd = (double *)malloc(sizeof(double)*nz);

    for (i = 0; i < n*T; i++) nu[i] = 0;
    for (i = 0; i < n*T; i++) b[i] = 0;

    F77_CALL(dgemv)("n",&n,&n,&fone,A,&n,x,&ione,&fzero,b,&ione);
    dptr = z; dptr1 = z0;
    for (i = 0; i < nz; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    if (quiet == 0)
    {
        printf("\n iteration \t step \t\t rd \t\t\t rp\n");
    }
    for (iiter = 0; iiter < maxiter; iiter++)
    {
        gfgphp(Q,R,Qf,zmax,zmin,z,T,n,m,nz,gf,gp,hp);
        rdrp(A,B,Q,R,Qf,z,nu,gf,gp,b,T,n,m,nz,kappa,rd,rp,Ctnu);
        resdresp(rd,rp,T,n,nz,&resd,&resp,&res);

        if (res < tol) break;

        dnudz(A,B,At,Bt,eyen,eyem,Q,R,Qf,hp,rd,rp,T,n,m,nz,kappa,dnu,dz);

        s = 1;
        /* feasibility search */
        while (1)
        {
            cont = 0;
            dptr = z; dptr1 = dz; dptr2 = zmax; dptr3 = zmin;
            for (i = 0; i < nz; i++)
            {
                if (*dptr+s*(*dptr1) >= *dptr2) cont = 1;
                if (*dptr+s*(*dptr1) <= *dptr3) cont = 1;
                dptr++; dptr1++; dptr2++; dptr3++;
            }
            if (cont == 1)
            {
                s = beta*s;
                continue;
            }
            else
                break;
        }

        dptr = newnu; dptr1 = nu; dptr2 = dnu;
        for (i = 0; i < T*n; i++)
        {
            *dptr = *dptr1+s*(*dptr2);
            dptr++; dptr1++; dptr2++;
        }
        dptr = newz; dptr1 = z; dptr2 = dz;
        for (i = 0; i < nz; i++)
        {
            *dptr = *dptr1+s*(*dptr2);
            dptr++; dptr1++; dptr2++;
        }

        /* insert backtracking line search */
        while (1)
        {
            gfgphp(Q,R,Qf,zmax,zmin,newz,T,n,m,nz,newgf,newgp,newhp);
            rdrp(A,B,Q,R,Qf,newz,newnu,newgf,newgp,b,T,n,m,nz,kappa,newrd,newrp,newCtnu);
            resdresp(newrd,newrp,T,n,nz,&newresd,&newresp,&newres);
            if (newres <= (1-alpha*s)*res) break;
            s = beta*s;
            dptr = newnu; dptr1 = nu; dptr2 = dnu;
            for (i = 0; i < T*n; i++)
            {
                *dptr = *dptr1+s*(*dptr2);
                dptr++; dptr1++; dptr2++;
            }
            dptr = newz; dptr1 = z; dptr2 = dz;
            for (i = 0; i < nz; i++)
            {
                *dptr = *dptr1+s*(*dptr2);
                dptr++; dptr1++; dptr2++;
            }
        }

        dptr = nu; dptr1 = newnu;
        for (i = 0; i < T*n; i++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = z; dptr1 = newz;
        for (i = 0; i < nz; i++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        if (quiet == 0)
        {
            printf("    %d \t\t %5.4f \t %0.5e \t\t %0.5e\n",iiter,s,newresd,newresp);
        }
    }
    dptr = z0; dptr1 = z;
    for (i = 0; i < nz; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }

    free(b); free(dnu); free(dz); free(nu); free(Ctnu);
    free(z); free(gp); free(hp); free(gf); free(rp); free(rd);
    free(newnu); free(newz); free(newCtnu); free(newgf); free(newhp);
    free(newgp); free(newrp); free(newrd);
    return;
}

/* computes the search directions dz and dnu */
void dnudz(double *A, double *B, double *At, double *Bt, double *eyen,
        double *eyem, double *Q, double *R, double *Qf, double *hp, double *rd,
        double *rp, int T, int n, int m, int nz, double kappa, double *dnu, double *dz)
{
    int i,j,info;
    double *dptr, *dptr1, *dptr2, *dptr3, *temp, *tempmatn, *tempmatm;
    double *PhiQ, *PhiR, *Yd, *Yud, *Ld, *Lld, *Ctdnu, *gam, *v, *be, *rdmCtdnu;
    double *PhiinvQAt, *PhiinvRBt, *PhiinvQeye, *PhiinvReye, *CPhiinvrd;
    //nT = n*T;

    /* allocate memory */
    PhiQ = (double *)malloc(sizeof(double)*n*n*T);
    PhiR = (double *)malloc(sizeof(double)*m*m*T);
    PhiinvQAt = (double *)malloc(sizeof(double)*n*n*T);
    PhiinvRBt = (double *)malloc(sizeof(double)*m*n*T);
    PhiinvQeye = (double *)malloc(sizeof(double)*n*n*T);
    PhiinvReye = (double *)malloc(sizeof(double)*m*m*T);
    CPhiinvrd = (double *)malloc(sizeof(double)*n*T);
    Yd = (double *)malloc(sizeof(double)*n*n*T);
    Yud = (double *)malloc(sizeof(double)*n*n*(T-1));
    Ld = (double *)malloc(sizeof(double)*n*n*T);
    Lld = (double *)malloc(sizeof(double)*n*n*(T-1));
    gam = (double *)malloc(sizeof(double)*n*T);
    v = (double *)malloc(sizeof(double)*n*T);
    be = (double *)malloc(sizeof(double)*n*T);
    temp = (double *)malloc(sizeof(double)*n);
    tempmatn = (double *)malloc(sizeof(double)*n*n);
    tempmatm = (double *)malloc(sizeof(double)*m*m);
    Ctdnu = (double *)malloc(sizeof(double)*nz);
    rdmCtdnu = (double *)malloc(sizeof(double)*nz);

    /* form PhiQ and PhiR */
    for (i = 0; i < T-1; i++)
    {
        dptr = PhiQ+n*n*i; dptr1 = Q;
        for (j = 0; j < n*n; j++)
        {
            *dptr = 2*(*dptr1);
            dptr++; dptr1++;
        }
        dptr = PhiQ+n*n*i; dptr1 = hp+m*(i+1)+n*i;
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr+kappa*(*dptr1);
            dptr = dptr+n+1; dptr1++;
        }
        dptr = PhiR+m*m*i; dptr1 = R;
        for (j = 0; j < m*m; j++)
        {
            *dptr = 2*(*dptr1);
            dptr++; dptr1++;
        }
        dptr = PhiR+m*m*i; dptr1 = hp+i*(n+m);
        for (j = 0; j < m; j++)
        {
            *dptr = *dptr+kappa*(*dptr1);
            dptr = dptr+m+1; dptr1++;
        }
    }

    dptr = PhiR+m*m*(T-1); dptr1 = R;
    for (j = 0; j < m*m; j++)
    {
        *dptr = 2*(*dptr1);
        dptr++; dptr1++;
    }
    dptr = PhiR+m*m*(T-1); dptr1 = hp+(T-1)*(n+m);
    for (j = 0; j < m; j++)
    {
        *dptr = *dptr+kappa*(*dptr1);
        dptr = dptr+m+1; dptr1++;
    }
    dptr = PhiQ+n*n*(T-1); dptr1 = Qf;
    for (j = 0; j < n*n; j++)
    {
        *dptr = 2*(*dptr1);
        dptr++; dptr1++;
    }
    dptr = PhiQ+n*n*(T-1); dptr1 = hp+m*T+n*(T-1);
    for (j = 0; j < n; j++)
    {
        *dptr = *dptr+kappa*(*dptr1);
        dptr = dptr+n+1; dptr1++;
    }

    /* compute PhiinvQAt, PhiinvRBt, PhiinvQeye, PhiinvReye */
    for (i = 0; i < T; i++)
    {
        dptr = PhiinvQAt+n*n*i; dptr1 = At;
        for (j = 0; j < n*n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n*n; dptr1 = PhiQ+n*n*i;
        F77_CALL(dposv)("l",&n,&n,dptr1,&n,dptr,&n,&info);
        dptr = PhiinvQeye+n*n*i; dptr1 = eyen;
        for (j = 0; j < n*n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n*n; dptr1 = PhiQ+n*n*i;
        F77_CALL(dtrtrs)("l","n","n",&n,&n,dptr1,&n,dptr,&n,&info);
        F77_CALL(dtrtrs)("l","t","n",&n,&n,dptr1,&n,dptr,&n,&info);
    }
    for (i = 0; i < T; i++)
    {
        dptr = PhiinvRBt+m*n*i; dptr1 = Bt;
        for (j = 0; j < n*m; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-m*n; dptr1 = PhiR+m*m*i;
        F77_CALL(dposv)("l",&m,&n,dptr1,&m,dptr,&m,&info);
        dptr = PhiinvReye+m*m*i; dptr1 = eyem;
        for (j = 0; j < m*m; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-m*m; dptr1 = PhiR+m*m*i;
        F77_CALL(dtrtrs)("l","n","n",&m,&m,dptr1,&m,dptr,&m,&info);
        F77_CALL(dtrtrs)("l","t","n",&m,&m,dptr1,&m,dptr,&m,&info);
    }

    /* form Yd and Yud */
    dptr = Yud; dptr1 = PhiinvQAt;
    for (i = 0; i < n*n*(T-1); i++)
    {
        *dptr = -(*dptr1);
        dptr++; dptr1++;
    }
    dptr2 = Yd; dptr3 = PhiinvQeye;
    for (i = 0; i < n*n; i++)
    {
        *dptr2 = *dptr3;
        dptr2++; dptr3++;
    }
    dptr2 = dptr2-n*n;
    F77_CALL(dgemm)("n","n",&n,&n,&m,&fone,B,&n,PhiinvRBt,&m,&fone,dptr2,&n);
    for (i = 1; i < T; i++)
    {
        dptr = Yd+n*n*i; dptr1 = PhiinvQeye+n*n*i;
        for (j = 0; j < n*n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr1 = PhiinvRBt+m*n*i; dptr = dptr-n*n;
        F77_CALL(dgemm)("n","n",&n,&n,&m,&fone,B,&n,dptr1,&m,&fone,dptr,&n);
        dptr1 = PhiinvQAt+n*n*(i-1);
        F77_CALL(dgemm)("n","n",&n,&n,&n,&fone,A,&n,dptr1,&n,&fone,dptr,&n);
    }

    /* compute Lii */
    dptr = Ld; dptr1 = Yd;
    for (i = 0; i < n*n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = dptr-n*n;
    F77_CALL(dposv)("l",&n,&ione,dptr,&n,temp,&n,&info);
    for (i = 1; i < T; i++)
    {
        dptr = Ld+n*n*(i-1); dptr1 = Yud+n*n*(i-1); dptr2 = Lld+n*n*(i-1);
        for (j = 0; j < n*n; j++)
        {
            *dptr2 = *dptr1;
            dptr2++; dptr1++;
        }
        dptr2 = dptr2-n*n;
        F77_CALL(dtrtrs)("l","n","n",&n,&n,dptr,&n,dptr2,&n,&info);
        dptr1 = tempmatn;
        for (j = 0; j < n*n; j++)
        {
            *dptr1 = *dptr2;
            dptr1++; dptr2++;
        }
        dptr1 = dptr1-n*n; dptr2 = dptr2-n*n;
        F77_CALL(dgemm)("t","n",&n,&n,&n,&fone,dptr1,&n,eyen,&n,&fzero,dptr2,&n);
        dptr = Ld+n*n*i; dptr1 = Yd+n*n*i;
        for (j = 0; j < n*n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n*n;
        F77_CALL(dgemm)("n","t",&n,&n,&n,&fmone,dptr2,&n,dptr2,&n,&fone,dptr,&n);
        F77_CALL(dposv)("l",&n,&ione,dptr,&n,temp,&n,&info);
    }

    /* compute CPhiinvrd */
    dptr = CPhiinvrd; dptr1 = rd+m;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = dptr-n;
    F77_CALL(dtrsv)("l","n","n",&n,PhiQ,&n,dptr,&ione);
    F77_CALL(dtrsv)("l","t","n",&n,PhiQ,&n,dptr,&ione);
    dptr2 = temp; dptr1 = rd;
    for (i = 0; i < m; i++)
    {
        *dptr2 = *dptr1;
        dptr2++; dptr1++;
    }
    dptr2 = dptr2-m;
    F77_CALL(dtrsv)("l","n","n",&m,PhiR,&m,dptr2,&ione);
    F77_CALL(dtrsv)("l","t","n",&m,PhiR,&m,dptr2,&ione);
    F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,temp,&ione,&fone,dptr,&ione);

    for (i = 1; i < T; i++)
    {
        dptr = CPhiinvrd+n*i; dptr1 = rd+m+i*(n+m);
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n; dptr3 = PhiQ+n*n*i;
        F77_CALL(dtrsv)("l","n","n",&n,dptr3,&n,dptr,&ione);
        F77_CALL(dtrsv)("l","t","n",&n,dptr3,&n,dptr,&ione);
        dptr2 = temp; dptr1 = rd+i*(m+n);
        for (j = 0; j < m; j++)
        {
            *dptr2 = *dptr1;
            dptr2++; dptr1++;
        }
        dptr3 = PhiR+m*m*i; dptr2 = dptr2-m;
        F77_CALL(dtrsv)("l","n","n",&m,dptr3,&m,dptr2,&ione);
        F77_CALL(dtrsv)("l","t","n",&m,dptr3,&m,dptr2,&ione);
        F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,temp,&ione,&fone,dptr,&ione);
        dptr2 = temp; dptr1 = rd+(i-1)*(n+m)+m;
        for (j = 0; j < n; j++)
        {
            *dptr2 = *dptr1;
            dptr2++; dptr1++;
        }
        dptr3 = PhiQ+n*n*(i-1); dptr2 = dptr2-n;
        F77_CALL(dtrsv)("l","n","n",&n,dptr3,&n,dptr2,&ione);
        F77_CALL(dtrsv)("l","t","n",&n,dptr3,&n,dptr2,&ione);
        F77_CALL(dgemv)("n",&n,&n,&fmone,A,&n,temp,&ione,&fone,dptr,&ione);
    }

    /* form be */
    dptr = be; dptr1 = rp; dptr2 = CPhiinvrd;
    for (i = 0; i < n*T; i++)
    {
        *dptr = (*dptr2)-(*dptr1);
        dptr++; dptr1++; dptr2++;
    }

    /* solve for dnu */
    dptr = v; dptr1 = be;
    for (i = 0; i < n; i++)
    {
        *dptr = -(*dptr1);
        dptr++; dptr1++;
    }
    dptr = dptr-n;
    F77_CALL(dtrsv)("l","n","n",&n,Ld,&n,dptr,&ione);
    for (i = 1; i < T; i++)
    {
        dptr = v+i*n; dptr1 = v+(i-1)*n; dptr2 = be+i*n;
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr2;
            dptr++; dptr2++;
        }
        dptr = dptr-n; dptr3 = Lld+n*n*(i-1);
        F77_CALL(dgemv)("n",&n,&n,&fmone,dptr3,&n,dptr1,&ione,&fmone,dptr,&ione);
        dptr3 = Ld+n*n*i;
        F77_CALL(dtrsv)("l","n","n",&n,dptr3,&n,dptr,&ione);
    }
    dptr = dnu+n*(T-1); dptr1 = v+n*(T-1);
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = dptr-n; dptr3 = Ld+n*n*(T-1);
    F77_CALL(dtrsv)("l","t","n",&n,dptr3,&n,dptr,&ione);
    for (i = T-1; i > 0; i--)
    {
        dptr = dnu+n*(i-1); dptr1 = dnu+n*i; dptr2 = v+n*(i-1);
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr2;
            dptr++; dptr2++;
        }
        dptr = dptr-n; dptr3 = Lld+n*n*(i-1);
        F77_CALL(dgemv)("t",&n,&n,&fmone,dptr3,&n,dptr1,&ione,&fone,dptr,&ione);
        dptr3 = Ld+n*n*(i-1);
        F77_CALL(dtrsv)("l","t","n",&n,dptr3,&n,dptr,&ione);
    }

    /* form Ctdnu */
    for (i = 0; i < T-1; i++)
    {
        dptr = Ctdnu+i*(n+m); dptr1 = dnu+i*n;
        F77_CALL(dgemv)("n",&m,&n,&fmone,Bt,&m,dptr1,&ione,&fzero,dptr,&ione);
        dptr = Ctdnu+i*(n+m)+m; dptr2 = dnu+(i+1)*n;
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n;
        F77_CALL(dgemv)("n",&n,&n,&fmone,At,&n,dptr2,&ione,&fone,dptr,&ione);
    }

    dptr = Ctdnu+(T-1)*(n+m); dptr1 = dnu+(T-1)*n;
    F77_CALL(dgemv)("n",&m,&n,&fmone,Bt,&m,dptr1,&ione,&fzero,dptr,&ione);
    dptr = dptr+m;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = rdmCtdnu; dptr1 = Ctdnu; dptr2 = rd;
    for (i = 0; i < nz; i++)
    {
        *dptr = -(*dptr1)-(*dptr2);
        dptr++; dptr1++; dptr2++;
    }

    /* solve for dz */
    for (i = 0; i < T; i++)
    {
        dptr = dz+(i+1)*m+i*n; dptr1 = rdmCtdnu+(i+1)*m+i*n;
        dptr2 = PhiinvQeye+n*n*i;
        F77_CALL(dgemv)("n",&n,&n,&fone,dptr2,&n,dptr1,&ione,&fzero,dptr,&ione);
    }
    for (i = 0; i < T; i++)
    {
        dptr = dz+i*(m+n); dptr1 = rdmCtdnu+i*(m+n);
        dptr2 = PhiinvReye+m*m*i;
        F77_CALL(dgemv)("n",&m,&m,&fone,dptr2,&m,dptr1,&ione,&fzero,dptr,&ione);
    }
    free(PhiQ); free(PhiR); free(PhiinvQAt); free(PhiinvRBt); free(PhiinvQeye);
    free(PhiinvReye); free(CPhiinvrd); free(Yd); free(Yud); free(Ld); free(Lld);
    free(gam); free(v); free(be); free(temp); free(tempmatn); free(tempmatm);
    free(Ctdnu); free(rdmCtdnu);
    return;
}

/* computes rd and rp */
void rdrp(double *A, double *B, double *Q, double *R, double *Qf, double *z, double *nu,
        double *gf, double *gp, double *b, int T, int n, int m, int nz,
        double kappa, double *rd, double *rp, double *Ctnu)
{
    int i, j;
    double *Cz;
    double *dptr, *dptr1, *dptr2;

    Cz = (double *)malloc(sizeof(double)*T*n);

    /* compute Cz */
    dptr = Cz; dptr1 = z+m;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,z,&ione,&fone,Cz,&ione);
    for (i = 2; i <= T; i++)
    {
        dptr = Cz+(i-1)*n; dptr1 = z+m+(i-2)*(n+m);
        dptr2 = z+m+(i-1)*(m+n);
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr2;
            dptr++; dptr2++;
        }
        dptr = dptr-n;
        F77_CALL(dgemv)("n",&n,&n,&fmone,A,&n,dptr1,&ione,&fone,dptr,&ione);
        dptr1 = dptr1+n;
        F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,dptr1,&ione,&fone,dptr,&ione);
    }
    /*
    dptr = Cz+(T-1)*n; dptr1 = z+m+(T-2)*(n+m);
    F77_CALL(dgemv)("n",&n,&n,&fmone,A,&n,dptr1,&ione,&fzero,dptr,&ione);
    dptr1 = dptr1+n;
    F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,dptr1,&ione,&fone,dptr,&ione);
    dptr1 = z+nz-n;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr+*dptr1;
        dptr++; dptr1++;
    }
    */

    /* compute Ctnu */
    dptr = Ctnu; dptr1 = Ctnu+m; dptr2 = nu;
    for (i = 1; i <= T-1; i++)
    {
        F77_CALL(dgemv)("t",&n,&m,&fmone,B,&n,dptr2,&ione,&fzero,dptr,&ione);
        dptr = dptr+n+m;
        for (j = 0; j < n; j++)
        {
            *dptr1 = *dptr2;
            dptr1++; dptr2++;
        }
        dptr1 = Ctnu+m+(i-1)*(n+m);
        F77_CALL(dgemv)("t",&n,&n,&fmone,A,&n,dptr2,&ione,&fone,dptr1,&ione);
        dptr1 = dptr1+n+m;
    }
    F77_CALL(dgemv)("t",&n,&m,&fmone,B,&n,dptr2,&ione,&fzero,dptr,&ione);
    dptr = Ctnu+nz-n; dptr1 = nu+(T-1)*n;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }

    dptr = rp; dptr1 = Cz; dptr2 = b;
    for (i = 0; i < n*T; i++)
    {
        *dptr = *dptr1-*dptr2;
        dptr++; dptr1++; dptr2++;
    }
    dptr = rd; dptr1 = Ctnu; dptr2 = gf;
    for (i = 0; i < nz; i++)
    {
        *dptr = *dptr1+*dptr2;
        dptr++; dptr1++; dptr2++;
    }
    F77_CALL(daxpy)(&nz,&kappa,gp,&ione,rd,&ione);
    free(Cz);
    return;
}

/* computes gf, gp and hp */
void gfgphp(double *Q, double *R, double *Qf, double *zmax, double *zmin, double *z,
        int T, int n, int m, int nz, double *gf, double *gp, double *hp)
{
    int i;
    double *dptr, *dptr1, *dptr2;
    double *gp1, *gp2;

    gp1 = (double *)malloc(sizeof(double)*nz);
    gp2 = (double *)malloc(sizeof(double)*nz);

    dptr = gp1; dptr1 = zmax; dptr2 = z;
    for (i = 0; i < nz; i++)
    {
        *dptr = 1.0/(*dptr1-*dptr2);
        dptr++; dptr1++; dptr2++;
    }
    dptr = gp2; dptr1 = zmin; dptr2 = z;
    for (i = 0; i < nz; i++)
    {
        *dptr = 1.0/(*dptr2-*dptr1);
        dptr++; dptr1++; dptr2++;
    }
    dptr = hp; dptr1 = gp1; dptr2 = gp2;
    for (i = 0; i < nz; i++)
    {
        *dptr = (*dptr1)*(*dptr1) + (*dptr2)*(*dptr2);
        dptr++; dptr1++; dptr2++;
    }
    dptr = gp; dptr1 = gp1; dptr2 = gp2;
    for (i = 0; i < nz; i++)
    {
        *dptr = *dptr1-*dptr2;
        dptr++; dptr1++; dptr2++;
    }

    dptr = gf; dptr1 = z;
    for (i = 0; i < T-1; i++)
    {
        F77_CALL(dgemv)("n",&m,&m,&ftwo,R,&m,dptr1,&ione,&fzero,dptr,&ione);
        dptr = dptr+m; dptr1 = dptr1+m;
        F77_CALL(dgemv)("n",&n,&n,&ftwo,Q,&n,dptr1,&ione,&fzero,dptr,&ione);
        dptr = dptr+n; dptr1 = dptr1+n;
    }
    F77_CALL(dgemv)("n",&m,&m,&ftwo,R,&m,dptr1,&ione,&fzero,dptr,&ione);
    dptr = dptr+m; dptr1 = dptr1+m;
    F77_CALL(dgemv)("n",&n,&n,&ftwo,Qf,&n,dptr1,&ione,&fzero,dptr,&ione);

    free(gp1); free(gp2);
    return;
}

/* computes resd, resp, and res */
void resdresp(double *rd, double *rp, int T, int n, int nz, double *resd,
        double *resp, double *res)
{
    int nnu = T*n;
    *resp = F77_CALL(dnrm2)(&nnu,rp,&ione);
    *resd = F77_CALL(dnrm2)(&nz,rd,&ione);
    *res = sqrt((*resp)*(*resp)+(*resd)*(*resd));
    return;
}
