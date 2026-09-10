"""Filter complex Gaussian amplitude, then square magnitude for correlated intensity."""
import numpy as np


def factors(shape, seed=0, correlation=(0.,0.), looks=1, contrast=1.):
    rng=np.random.default_rng(seed)
    fy=np.fft.fftfreq(shape[0])[:,None];fx=np.fft.fftfreq(shape[1])[None,:]
    transfer=np.exp(-2*np.pi**2*((correlation[0]*fy)**2+(correlation[1]*fx)**2))
    # Parseval normalization uses ensemble variance, not each frame's random mean.
    variance=np.mean(transfer**2)
    intensity=np.zeros(shape)
    for _ in range(looks):
        field=(rng.normal(size=shape)+1j*rng.normal(size=shape))/np.sqrt(2)
        field=np.fft.ifft2(np.fft.fft2(field)*transfer)/np.sqrt(variance)
        intensity+=np.abs(field)**2/looks
    return 1+contrast*(intensity-1)
