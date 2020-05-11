   function[Nbar]=rscale(a,b,c,d,k)
         % Given the single-input linear system:
         %       .
         %       x = Ax + Bu
         %       y = Cx + Du
         % and the feedback matrix K,
         %
         % the function rscale(sys,K) or rscale(A,B,C,D,K)
         % finds the scale factor N which will
         % eliminate the steady-state error to a step reference
         % for a continuous-time, single-input system
         % with full-state feedback using the schematic below:
         %
         %                         /---------\
         %      R         +     u  | .       |
         %      ---> N --->() ---->| X=Ax+Bu |--> y=Cx ---> y
         %                -|       \---------/
         %                 |             |
         %                 |<---- K <----|
         %
         %8/21/96 Yanjie Sun of the University of Michigan
         %        under the supervision of Prof. D. Tilbury
         %6/12/98 John Yook, Dawn Tilbury revised
         error(nargchk(2,5,nargin));
         % --- Determine which syntax is being used ---
         nargin1 = nargin;
         if (nargin1==2),	% System form
         		[A,B,C,D] = ssdata(a);
         		K=b;
         elseif (nargin1==5), % A,B,C,D matrices
         		A=a; B=b; C=c; D=d; K=k;
         else error('Input must be of the form (sys,K) or (A,B,C,D,K)')
         end;
         % compute Nbar
         s = size(A,1);
         Z = [zeros([1,s]) 1];
         N = inv([A,B;C,D])*Z';
         Nx = N(1:s);
         Nu = N(1+s);
         Nbar=Nu + K*Nx;