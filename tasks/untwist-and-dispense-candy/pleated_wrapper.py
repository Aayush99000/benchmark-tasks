"""Construct a formed twist wrap from a strip of constant material width."""
import numpy as np
from geometry import Surface, grid


def _section(y, samples, seam_angle=0.):
    """Form one cross-section. Pleats store the sheet width in each narrow tail."""
    u=np.linspace(0.,1.,1025)
    angle=2*np.pi*1.12*u+seam_angle
    shoulder=float(np.clip((abs(y)-.013)/.010,0.,1.))
    tail=float(np.clip((abs(y)-.023)/.017,0.,1.))
    rx=.0096*(1-shoulder)+(.0034+.0024*tail)*shoulder
    rz=.0063*(1-shoulder)+(.0034+.0015*tail)*shoulder
    turn=np.sign(y)*2*np.pi*(.65*shoulder+.35*tail)
    power=.28+.72*shoulder
    def profile(amplitude):
        x=np.sign(np.cos(angle))*np.abs(np.cos(angle))**power*(rx+.00045*u)
        z=np.sign(np.sin(angle))*np.abs(np.sin(angle))**power*(rz+.00045*u)
        delta=amplitude*np.cos(8*angle)
        x+=delta*np.cos(angle)
        z+=delta*np.sin(angle)
        return np.column_stack((x*np.cos(turn)-z*np.sin(turn),x*np.sin(turn)+z*np.cos(turn)))
    def sample(amplitude):
        points=profile(amplitude)
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
        # Sample at equal material arc lengths to avoid needle triangles near
        # the rounded corners. Work with the polygon the solver actually sees.
        target=np.linspace(0.,arc[-1],samples)
        return np.column_stack([np.interp(target,arc,points[:,j]) for j in (0,1)])
    low,high=0.,.0033
    for _ in range(24):
        mid=(low+high)/2
        points=sample(mid)
        if np.linalg.norm(np.diff(points,axis=0),axis=1).sum()<.067:
            low=mid
        else:
            high=mid
    return sample((low+high)/2)


def build(seam_angle=0.):
    n,m=65,73
    ys=np.linspace(-.04,.04,n)
    sections=np.stack([_section(y,m,seam_angle) for y in ys])
    points=np.empty((n,m,3))
    points[:,:,0]=.38+sections[:,:,0]
    points[:,:,1]=ys[:,None]
    points[:,:,2]=.7545+sections[:,:,1]
    surface=Surface('wrapper',points.reshape(-1,3),grid(n,m),(.85,.65,.16),
                    .000045,1000.,2e6,.05,.38,.00035)
    # A dead-fold material retains its formed creases. The rest *metric* now
    # contains the original transverse width in every longitudinal row.
    return surface

if __name__=='__main__':
    p=build().points.reshape(65,73,3)
    widths=np.linalg.norm(np.diff(p,axis=1),axis=2).sum(axis=1)
    print(widths.min(),widths.max(), p[:,:,2].min())
