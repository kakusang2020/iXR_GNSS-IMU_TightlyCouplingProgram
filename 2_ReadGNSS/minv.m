function xx = minv(xx,eps,nsv)

% %//---matrix inversion & determinant---//
% 	int work[4000],i,j,k,l,m,r,iw,s,t,u,v;
% 	double w,wmax,pivot,api,w1,det;
% //	extern double fabs();
% //	double a[]={xx[]};
% //	l = nsv*2+3;
% //	m = nsv*2+3;

l=nsv;
m=nsv;

%if(m<2 || m>500 || eps<=0.0)return(999);
w1 = 1.0;
work = aeros(1,m);
for i=1:m
    work(i)=i;
end

for k=1:m
    
    wmax=0.0;
    for i=k:m
        
        w=fabs(xx(i*l+k));
        if(w>wmax)
            
            wmax=w;
            r=i;
        end
    end
    pivot=xx(r*l+k);
    api=fabs(pivot);
    if(api<=eps)
        
        det=w1;
        %return(1);
    end
    w1= w1 * pivot;
    u=k*l;
    v=r*l;
    if(r~=k)
        
        w1=-w1;
        iw=work(k);
        work(k)=work(r);
        work(r)=iw;
        for j=1:m
            
            s=u+j;
            t=v+j;
            w=xx(s);
            xx(s)=xx(t);
            xx(t)=w;
        end
    end
    for i=1:m
        xx(u+i)= xx(u+i)/ pivot;
    end
    for i=1:m
        
        if(i~=k)
            
            v=i*l;
            s=v+k;
            w=xx(s);
            if(w~=0.0)
                
                for j=1:m
                    if(j~=k)
                        xx(v+j)=xx(v+j) - w*xx(u+j);
                    end
                    xx(s) = -w/pivot;
                    
                end
            end
        end
        xx(u+k)=1.0/pivot;
    end
    for i=1:m
        
        while(1)
            
            k=work(i);
            if(k==i)
                break;
            end
            iw=work(k);
            work(k)=work(i);
            work(i)=iw;
            for j=1:m
                
                u=j*l;
                s=u+i;
                t=u+k;
                w=xx(s);
                xx(s)=xx(t);
                xx(t)=w;
            end
        end
    end
    det=w1;
    
    %return(0);
end




end

