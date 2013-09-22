function Gravity1Simulation()
{
    this.mu = 0.5;  // M * G
    this.minR = 0.09;
    this.velpos = [0,1.2,0.5,0];
    this.solver = 'Trapezoidal';
    this.initialEnergy = 1+this.getKineticEnergy() + this.getPotentialEnergy();
}

function weightedsum(A,x,len)
{
    var result = [];
    var n = A.length;
    for (var i = 0; i < len; ++i)
    {
        var y = 0;
        for (var j = 0; j < n; ++j)
        {
            y += A[j]*x[j][i];
        }
        result.push(y);
    }
    return result;
}

function dotpath(ctx,x,y,r)
{
    ctx.beginPath();
    ctx.arc(x, y, r, 0, 2 * Math.PI, false);
}
function sumsq(x,y) { return x*x+y*y; }
function hypot(x,y) { return Math.sqrt(x*x+y*y); }
function drawbar(ctx,x0,dx,height,hmargin,e)
{
    var dy = e*(height-2*hmargin);
    var ystart = dy > 0 ? height-hmargin-dy : hmargin;
    ctx.fillRect(x0,ystart,dx,dy > 0 ? dy : -dy);
}

Gravity1Simulation.prototype = 
{
    // 4-vector containing velx, vely, posx, posy
    calcDerivative: function(vx)
    {
        var r = hypot(vx[2],vx[3])
        // |a| = F/M2 = 1/M2*(M1*M2*G/r/r) = M1*G/r/r
        // a = -|a|*x/|x| = -|a|*x/r
        var k = -this.mu/r/r/r;
        return [k*vx[2],k*vx[3],vx[0],vx[1]]; 
    },
    update: function(dt)
    {
        var S = this.solvers[this.solver];
        var me = this;
        var f = function(vx) { return me.calcDerivative(vx); }
        this.velpos = S(this.velpos, f, dt);
    },
    solvers: { 
        'Euler': function(vx, f, dt)
        {
            var dvxdt = f(vx);
            return weightedsum([1,dt], [vx,dvxdt], 4);
            // vx + dt*dvxdt
        },
        'Trapezoidal': function(vx, f, dt)
        {
            var dvxdt1 = f(vx);
            var vx1 = weightedsum([1,dt], [vx,dvxdt1], 4);
            var dvxdt2 = f(vx1);
            return weightedsum([1,dt/2,dt/2], [vx,dvxdt1,dvxdt2], 4);
        }
    },
    getRadius: function() {
        return hypot(this.velpos[2],this.velpos[3]);
    },
    getKineticEnergy: function() { return sumsq(this.velpos[0],this.velpos[1])/2; },
    getPotentialEnergy: function() {
        return this.mu*(-1.0/this.getRadius());
    },    
    getAngularMomentum: function() {
        // r x v = rx*vy - ry*vx
        return this.velpos[1]*this.velpos[2] - this.velpos[0]*this.velpos[3];
    },
    draw: function(canvas)
    {
        var ctx = canvas.getContext('2d');
        var cw = canvas.width;
        var ch = canvas.height;
        var rmax = Math.min(cw,ch)/2;
        
        var x1 = cw/2 + rmax*this.velpos[2];
        var y1 = ch/2 - rmax*this.velpos[3];
        
        ctx.fillStyle = '#000000';
        ctx.fillRect(0,0,cw,ch);
        
        ctx.fillStyle = '#ffffff';        
        dotpath(ctx, cw/2,ch/2,5);
        ctx.fill();        

        dotpath(ctx,x1,y1,2);
        ctx.fill();  
        
        var e0=0.2 / this.initialEnergy;
        var ke=e0*this.getKineticEnergy();
        var pe=e0*this.getPotentialEnergy();
        
        ctx.fillStyle = '#ff8080';
        drawbar(ctx,cw-13,2,ch,2,ke);        
        ctx.fillStyle = '#80ff80';
        drawbar(ctx,cw-9,2,ch,2,pe);   
        ctx.fillStyle = '#8080ff';
        drawbar(ctx,cw-5,2,ch,2,ke+pe);     
    },
    drawOrbitPath: function(canvas)
    {
        // based on http://ocw.mit.edu/courses/aeronautics-and-astronautics/16-346-astrodynamics-fall-2008/lecture-notes/lec_01.pdf
        var ctx = canvas.getContext('2d');
        var cw = canvas.width;
        var ch = canvas.height;
        var rmax = Math.min(cw,ch)/2;

        var h = this.getAngularMomentum();
        var r = this.getRadius();
        var mu_e = [this.velpos[1]*h  - this.mu/r*this.velpos[2],
                    -this.velpos[0]*h - this.mu/r*this.velpos[3]];
        var e = hypot(mu_e[0],mu_e[1])/this.mu;            
        ctx.beginPath();
        var penup = true;
        var th_start = 0;
        var th_stop = 1;
        var done = false;
        if (e > 1)
        {
            var rclosest = h*h/this.mu/(1+e);
            var th_c = Math.atan2(mu_e[1],mu_e[0]);
            var th_d = Math.PI-Math.acos(1/e);
            th_start = (th_c-th_d+0.00001)/2/Math.PI;
            th_stop = (th_c+th_d-0.00001)/2/Math.PI;
            var rc = rmax*rclosest*(1+1/(e-1));
            var rk = 2*rmax+rc;
            var cx = cw/2 + rc*Math.cos(th_c);
            var cy = ch/2 - rc*Math.sin(th_c);
            ctx.moveTo(cx + rk*Math.cos(th_c-th_d),
                       cy - rk*Math.sin(th_c-th_d));
            ctx.lineTo(cx,cy);
            ctx.lineTo(cx + rk*Math.cos(th_c+th_d),
                       cy - rk*Math.sin(th_c+th_d));
        }
        th = th_start;
        
        while (true)
        {
            var th_i = th * 2 * Math.PI;
            var costh = Math.cos(th_i);
            var sinth = Math.sin(th_i);
            var r_i = h*h/(this.mu + (mu_e[0]*costh + mu_e[1]*sinth));
            if (r_i < 0)
            {
                penup = true;
                break;
            }
            else
            {
                var rx = r_i*costh;
                var ry = r_i*sinth;
                var x = cw/2 + rmax*rx;
                var y = ch/2 - rmax*ry;
                if (penup)
                {
                    ctx.moveTo(x,y);
                    penup = false;
                }
                else
                    ctx.lineTo(x,y);
            }
            
            if (th >= th_stop)
                break;
            th += 0.05;
            if (th > th_stop)
                th = th_stop;
        }
    }
}

