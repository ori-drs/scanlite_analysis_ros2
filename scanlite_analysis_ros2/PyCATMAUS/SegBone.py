# # SegmentBone functions
# version: 1.0

# Converted from Matlab script to python version
# Bone Segmentation in US images 

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from scipy import ndimage
import time
import concurrent.futures

# BoneSegmentation functions
def FiltGaussian(shape,sigma):
# Gaussian filter - converted from matlab - should give the same result as MATLAB: fspecial('gaussian',[shape],[sigma])
    m,n = [(ss-1.)/2. for ss in shape]
    y,x = np.ogrid[-m:m+1,-n:n+1]
    h = np.exp( -(x*x + y*y) / (2.*sigma*sigma) )
    h[ h < np.finfo(h.dtype).eps*h.max() ] = 0
    sumh = h.sum()
    if sumh != 0:
        h /= sumh
    return h

def AdjContrast(im):
    imOut = (im-im.min())/((im.max()-im.min()) if im.max()-im.min() !=0 else 1)
    return imOut
# %%
def getEOP(bpFlt,F,H1,H2):
    eMG = np.real(np.fft.ifftn(F*np.fft.fftshift(bpFlt)))
    h1f = np.real(np.fft.ifftn(F*np.fft.fftshift(H1*bpFlt)))
    h2f = np.real(np.fft.ifftn(F*np.fft.fftshift(H2*bpFlt)))
    oMG = (h1f**2 + h2f**2)**0.5
    pha = np.arctan2(oMG,eMG)
    return eMG, oMG, pha

def get_eMG(bpFlt,F):
    eMG = np.real(np.fft.ifftn(F*np.fft.fftshift(bpFlt)))
    return eMG
def get_h1f(bpFlt,F,H1):
    h1f = np.real(np.fft.ifftn(F*np.fft.fftshift(H1*bpFlt)))
    return h1f
def get_h2f(bpFlt,F,H2):
    h2f = np.real(np.fft.ifftn(F*np.fft.fftshift(H2*bpFlt)))
    return h2f
#img,sigma,filtType,Tsc = imBL*255,[50, 100, 150],'lg',0.18 #### to remove
# %%
def FeatureAsymmetry2D(img,sigma,filtType,Tsc):
    ''' 
    Matlab Function: Feature Asymmetry2D(img, sigma, filtType, Tsc)
    variables input in SegmentBone.m are: (imBLOG.*255), [50 100 150] , 'lg', 0.18)
    '''
    # %%
    # ti=time.perf_counter()
    [xdim, ydim] = np.shape(img)
    numSigmas = len(sigma)
    xEven = xdim % 2
    yEven = ydim % 2
    # ensuring that we always have even-dimension images
    xEndPix = 0 - xEven
    yEndPix = 0 - yEven
    image = img[0:xdim+xEndPix, 0:ydim+yEndPix]
    # Compute 2D monogenic signal
    # [evenMG, oddMG, phase] = monogenic2D(image, wavelengths, filtType)
    ##--- Matlab function monogenic2D starts---##
    szX = xdim + xEndPix
    szY = ydim + yEndPix
    szXmid = int(szX / 2)
    szYmid = int(szY / 2)
    xGrid, yGrid = np.mgrid[-szXmid:szXmid,-szYmid:szYmid]
    xGrid = xGrid / szX
    yGrid = yGrid / szY
    w = (xGrid**2 + yGrid**2)**0.5
    w[szXmid, szYmid] = 1
    sigmaOnf = 0.5 ### come back to check why
    # tb1=time.perf_counter()    
    bpFilt=[]
    FILTER = {
        'lg': lambda w,f0,w0,flt: np.exp((-(np.log(w/w0))**2) / (2 * np.log(sigmaOnf)**2)),
        'gabor': lambda w,f0,w0,flt: np.exp((-(w/w0)**2) / (2 * sigmaOnf**2)),
        'gd': lambda w,f0,w0,flt: w * np.exp(-(w**2)*(sigma[flt]**2)),
        'cau': lambda w,f0,w0,flt: w * np.exp(-(w)*(sigma[flt]))}
    for flt in range(0,numSigmas):
        f0 = 1/sigma[flt]
        w0 = f0/0.5
        R = FILTER[filtType](w,f0,w0,flt)
        R[szXmid, szYmid]=0
        bpFilt.append(R)
    sumFilt = bpFilt[0]
    for flt in range(1,numSigmas):
        sumFilt += bpFilt[flt]
    for flt in range(0,numSigmas):
        bpFilt[flt] = bpFilt[flt] /sumFilt.max()
    # tb2=time.perf_counter() 
    # generating the Riesz filter components (i.e. the odd filter whose
    # components are imaginary)
    H1 = complex(0,1) * xGrid / w
    H2 = complex(0,1) * yGrid / w
 
    F = np.fft.fftn(image)
    evenMG = []
    H1f = []
    H2f = []
    oddMG = []
    phase = []
    # %%
    ''' replaced by concurrent processing
    for flt in range(0,numSigmas):
    # computing the parts of the monogenic signal (taking care to remove
    # the DC component by shifting the zero-frequency component of the
    # bandpass filters to the center of the spectrum
        eMG = np.real(np.fft.ifftn(F*np.fft.fftshift(bpFilt[flt])))
        evenMG.append(eMG) #even component
        h1f = np.real(np.fft.ifftn(F*np.fft.fftshift(H1 * bpFilt[flt])))
        #H1f.append(h1f)    #odd components...
        h2f = np.real(np.fft.ifftn(F*np.fft.fftshift(H2*bpFilt[flt])))
        #H2f.append(h2f)
        #oMG = (H1f[flt]**2 + H2f[flt]**2)**0.5
        oMG = (h1f**2 + h2f**2)**0.5
        oddMG.append(oMG)
        pha = np.arctan2(oMG,eMG)
        phase.append(pha)  # phase = (imaginary/real) parts of the signal
    '''
    with concurrent.futures.ThreadPoolExecutor() as executor:
        E_result=[executor.submit(get_eMG,bp,F) for bp in bpFilt]
        h1_result=[executor.submit(get_h1f,bp,F,H1) for bp in bpFilt]
        h2_result=[executor.submit(get_h2f,bp,F,H2) for bp in bpFilt]
        for i in range(numSigmas):
            eMG= E_result[i].result()
            h1f= h1_result[i].result()
            h2f= h2_result[i].result()
            oMG = (h1f**2 + h2f**2)**0.5
            pha = np.arctan2(oMG,eMG)
            evenMG.append(eMG)
            oddMG.append(oMG)
            phase.append(pha)
        # EOP_result=[executor.submit(getEOP,bp,F,H1,H2) for bp in bpFilt]
        # for i in range(numSigmas):
        #     eMG, oMG, pha =EOP_result[i].result()
        #     evenMG.append(eMG)
        #     oddMG.append(oMG)
        #     phase.append(pha)
    #--- Matlab function monogenic2D ends
    # tb3=time.perf_counter() 
    epsilon = 0.001 # to come back to check reason
    #FA = np.zeros([szX,szY])
    FS = np.zeros([szX,szY])
    for i in range(0, numSigmas):
        odd_mg = oddMG[i]
        even_mg = evenMG[i]
        LE = (even_mg**2 + odd_mg**2)** 0.5
        #generate local energy image acatually local amplitude
        numeratorFS = np.abs(even_mg)-np.abs(odd_mg)-Tsc
        denominator = epsilon + (odd_mg**2 + even_mg**2)**0.5
        FS0ind = numeratorFS > 0 ##### RuntimeWarning: invalid value encountered in greater
        FS[FS0ind]+= numeratorFS[FS0ind]/denominator[FS0ind] 
        # numeratorFA = np.abs(odd_mg)-np.abs(even_mg)-Tsc
        # FA0ind = numeratorFS>0
        # FA[FA0ind]=FA[FA0ind]+(numeratorFA[FA0ind]/denominator[FA0ind])           
    #FA = FA/numSigmas
    #FA = np.pad(FA,((0,np.abs(xEndPix)),(0,np.abs(yEndPix))),'constant')
    FS = FS/numSigmas
    FS = np.pad(FS,((0,np.abs(xEndPix)),(0,np.abs(yEndPix))),'constant')
    LP = np.pi-phase[1]
    LP = np.pad(LP,((0,np.abs(xEndPix)),(0,np.abs(yEndPix))),'constant')
    LE = np.pad(LE,((0,np.abs(xEndPix)),(0,np.abs(yEndPix))),'constant')
    # tb4=time.perf_counter()
    # print(f'B1: {tb1-ti:1.3} sec')
    # print(f'B2: {tb2-tb1:1.3} sec')
    # print(f'B3: {tb3-tb2:1.3} sec')
    # print(f'B4: {tb4-tb3:1.3} sec') 
    # %%
    return LP, LE,  FS #,FA

def SBDP(Bness, F0, F1, Bth, JumpConst):
    '''Find bone surface using Dynamic Programming
    # - Smoothness will be applied through DP
    # - Presence of bone will be also determined using DP
    # matlab function: imSeg = SegmentBoneDP(Bness,F0,F1,Bth,JumpConst)
    '''
    [bH, bW] = Bness.shape
    Ecnct = -0.1* Bth #-0.01 # ???
    CostF = 1000 * np.ones((bH, bW))       # The total cost for each point
    mini = bH * np.ones((bH, bW))          # The index of minimum cost path from each point
    diffF = JumpConst * np.ones((bH, bW))  # The slop of path at each point (for smoothness)
    # The last row is used as "no bone" pass.
    Bness[bH-1, :] = Bth                   # use a threshold for no bone
    ScLine = np.arange(1,bH)
    for ww in range(1,bW):#23
        for hh in range(0,bH-1):#24
            if Bness[hh,ww] > 0.1:#25
                Cdiff = (ScLine-hh)/bH
                Esmth = (Cdiff-diffF[0:bH-1,ww-1])**2
                Econt = Cdiff**2
                Etotal = F0*Econt + F1*Esmth + Ecnct
                CFarray = CostF[0:bH-1,ww-1] + Etotal
                minCF = CFarray.min()
                mini[hh,ww]=CFarray.argmin()
                        
                Etotal = JumpConst # should be constant
                CF = CostF[bH-1, ww-1] + Etotal            
                if CF<minCF:#48
                    minCF = CF
                    mini[hh,ww] = bH
                    diffF[hh,ww] = 0
                else:
                    diffF[hh,ww] = (mini[hh,ww]-hh+1)/bH
                    
                CostF[hh,ww] = (1-Bness[hh,ww])+minCF
            else:
                mini[hh,ww] = bH
                CostF[hh,ww] = 1000 + CostF[hh, ww-1]
        #hh = bH-1 #ln65 for hh=bH
        minCF = 100000
        Etotal = JumpConst/(bH**2)
        CFarray= CostF[0:bH-1,ww-1] + Etotal
        minCF = CFarray.min()
        mini[-1,ww] = CFarray.argmin()
        
        Etotal = (F0+F1)/(bH**2)
        CF = CostF[-1,ww-1] + Etotal
        if CF<minCF: #75
            minCF = CF
            mini[-1,ww] = bH
        CostF[-1,ww] = 1-Bness[-1,ww] + minCF    
        diffF[-1,ww] = 0
    #end of for loop in matlab ln 23- 87
    ##### to consider return cordinates instead of image array  #####     
    indSeg = np.zeros(bW)
    indSeg[bW-1] = bH
    for ii in range(bW-2,-1,-1):
        idx = np.int32(indSeg[ii])
        indSeg[ii-1] = mini[idx-1,ii-1]
    
    imSeg = np.zeros((bH,bW))
    for ii in range(0,bW):
        imSeg[int(indSeg[ii])-1,ii-1] = 1
    return imSeg

def RunBoneSeg(img,F0,F1,Bth,JC):
    '''
    funciton to simplify realtime display coding
    added: 21/11/19

    '''
    [imH, imW] = np.shape(img)
    # these parameters may need some adjustments
    USdepth = 0.04
    boneTh = 0.1 # this is for masking, which may need to be adjusted later on
    Gsigma = 6
    hsize = int((Gsigma*3)*2+1)
    #%3
    h = FiltGaussian((hsize,hsize),Gsigma)
    imBlured = sp.ndimage.correlate(img, h, mode = 'nearest') 
    imBlured = AdjContrast(imBlured)
    #%4
    h = [[0,-1, 0],[-1, 4, -1],[0, -1, 0]]
    imBL = sp.ndimage.correlate(imBlured, h, mode = 'nearest')
    imBL[imBL < 0] = 0 
    imBL = imBL/np.amax(imBL) if np.amax(imBL) != 0 else imBL
    #%5
    imMask = imBlured >= boneTh
    BorderRigeon = round((0.001/USdepth)*imH)
    imMask[0:BorderRigeon][:] = 0
    #%6    
    Gsigma = 2 ### come back to check this value
    tt = np.arange(1,imH+1)
    ShadowM =1-( np.exp(-(tt-1)**2/(2*Gsigma**2)))
    #%7
    imMask[imH-1,:] = 0
    Bsh = np.ones((imH,imW))
    for k in range(0,imH-1):
        ul = np.double(img[k:,:])
        ulsize = len(ul)
        SM = ShadowM[0:ulsize]
        SM/= np.sum(SM[:])
        Bsh[k,:] = np.sum(SM.reshape(ulsize,1)*ul,axis=0)/ulsize
    Bsh = 1-AdjContrast(Bsh)
    Wshw = Bsh **2
    # calculate the intensity weight
    Wint = imMask * (imBlured+imBL)
    Wint = AdjContrast(Wint)
    # calculate the total Boniness, (Multipication prefered over addition) 
    BnessOrin = Wshw*Wint
    BnessOrin = BnessOrin/ BnessOrin.max() if (BnessOrin.max() != 0) else BnessOrin
    #%8
    imad = np.zeros((imH,imW))
    for r in range(0,imH):
        imad[r,:] = np.sum(img[0:r,:],axis=0)
    imad = AdjContrast(imad)
    imad = imad*imMask
    #%9
    LP, LE, FS = FeatureAsymmetry2D(imBL*255,[50, 100, 150],'lg',0.18)
    LP = AdjContrast(LP)
    FS = AdjContrast(FS)
    LE = AdjContrast(LE)
    LI = LE*LP*FS
    LI = AdjContrast(LI)
    #%10
    LBoost = LI*imad
    LBoost = AdjContrast(LBoost)
    # Bone probablility
    Bness = LBoost* BnessOrin
    imSeg = SBDP(Bness,F0,F1,Bth,JC)
    [r, c] = np.nonzero(imSeg)
    IdxBtmLine = r != imH-1
    coor = [c[IdxBtmLine],r[IdxBtmLine]]
    return coor

# %%
