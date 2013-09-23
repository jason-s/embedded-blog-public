function Gravity1Simulation()
{
    this.mu = 0.5;  // M * G
    this.minR = 0.09;
    this.velpos = [0.4,-0.4,0.66,0.66];
    this.solver = 'Trapezoidal';
    this.initialEnergy = 1+this.getKineticEnergy() + this.getPotentialEnergy();
    this.thrusterstrength = 0.05;
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
    thruster: function(keymap, dt)
    {
        if (keymap.up)
            this.velpos[1] += this.thrusterstrength * dt;
        if (keymap.down)
            this.velpos[1] -= this.thrusterstrength * dt;
        if (keymap.right)
            this.velpos[0] += this.thrusterstrength * dt;
        if (keymap.left)
            this.velpos[0] -= this.thrusterstrength * dt;
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
    getSpeed: function() { return hypot(this.velpos[0],this.velpos[1]); },
    getAngularMomentum: function() {
        // r x v = rx*vy - ry*vx
        return this.velpos[1]*this.velpos[2] - this.velpos[0]*this.velpos[3];
    },
    getEccentricity: function() {
        var mu = this.mu;
        var h = this.getAngularMomentum();
        var r = this.getRadius();
        var mu_e = [this.velpos[1]*h  - mu/r*this.velpos[2],
                    -this.velpos[0]*h - mu/r*this.velpos[3]];
        return hypot(mu_e[0],mu_e[1])/mu;     
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
        ctx.beginPath();
        dotpath(ctx, cw/2,ch/2,5);
        ctx.fill();        

        ctx.beginPath();
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
        var mu = this.mu;
        var mu_e = [this.velpos[1]*h  - mu/r*this.velpos[2],
                    -this.velpos[0]*h - mu/r*this.velpos[3]];
        var e = hypot(mu_e[0],mu_e[1])/mu;            
        ctx.beginPath();
        var th_start = 0;
        var th_stop = 2*Math.PI;
        if (e > 1)
        {
            var rclosest = h*h/mu/(1+e);
            var th_c = Math.atan2(mu_e[1],mu_e[0]);
            var th_d = Math.PI-Math.acos(1/e);
            th_start = th_c-th_d+0.00001;
            th_stop = th_c+th_d-0.00001;
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
        else if (e > 0.05)
        {
            var th_c = Math.atan2(mu_e[1],mu_e[0]);
            th_start = th_c - Math.PI;
            th_stop = th_c + Math.PI;            
        }
        th = th_start;
        
        function f(theta)
        {
            var costh = Math.cos(theta);
            var sinth = Math.sin(theta);
            var r_i = h*h/(mu + (mu_e[0]*costh + mu_e[1]*sinth));
            var rx = r_i*costh;
            var ry = r_i*sinth;
            var x = cw/2 + rmax*rx;
            var y = ch/2 - rmax*ry;
            return [x,y];
        }
        var N = 16;
        for (var i = 0; i <= N; ++i)
        {   
            var rho = i / N;
            var rhowarp = rho - 0.04*Math.sin(2*Math.PI*rho);
            var th_i = th_start + rhowarp*(th_stop-th_start);
            var xy = f(th_i);
            /*if (i == 0)
            {
                ctx.moveTo(xy[0],xy[1]);
                dotpath(ctx,xy[0],xy[1],2);
            }
            else
                ctx.lineTo(xy[0],xy[1]);
            if (i == N)*/
                dotpath(ctx,xy[0],xy[1],2);
        }
    }
}

