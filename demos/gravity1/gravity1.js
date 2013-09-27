function Gravity1Simulation()
{
    this.mu = 0.5;  // M * G
    this.minR = 0.09;
    this.velpos = [0.4,-0.4,0.66,0.66];
    this.solver = 'Trapezoidal';
    this.initialEnergy = 1+this.getKineticEnergy() + this.getPotentialEnergy();
    this.thrusterstrength = 0.05;
    this.thrusterAcceleration = [0,0];
    this.krhowarp = 0;
    this.drawScale = 0;
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
function arrowpath(ctx,x,y,dx,dy)
{
    ctx.moveTo(x,y);
    var d = hypot(dx,dy);
    if (d > 1e-9)
    {
      var ux=dx/d;
      var uy=dy/d;
      ctx.lineTo(x+dx,y+dy);
      var hw = 5;
      var hl = 10;
      ctx.lineTo(x+dx-hw*uy-hl*ux,y+dy+hw*ux-hl*uy);
      ctx.lineTo(x+dx,y+dy);
      ctx.lineTo(x+dx+hw*uy-hl*ux,y+dy-hw*ux-hl*uy);
    }
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
    calcDerivative: function(vx, etc)
    {
        var r = hypot(vx[2],vx[3])
        // Acceleration w/o additional force:
        // |a| = F/M2 = 1/M2*(M1*M2*G/r/r) = M1*G/r/r
        // a = -|a|*x/|x| = -|a|*x/r
        var k = -this.mu/r/r/r;
        var ax = k*vx[2] + etc.thrusterAcceleration[0];
        var ay = k*vx[3] + etc.thrusterAcceleration[1];
        return [ax,ay,vx[0],vx[1]]; 
    },
    update: function(dt)
    {
        var S = this.solvers[this.solver];
        var me = this;
        var etc = {thrusterAcceleration: this.thrusterAcceleration};
        var f = function(vx) { return me.calcDerivative(vx, etc); }
        this.velpos = S(this.velpos, f, dt);
    },
    updateControls: function(keystate, keypress)
    {
        newScale = this.drawScale;
        if (keypress['-'])
            ++newScale;
        if (keypress['+'])
            --newScale;
        newScale = Math.min(20,Math.max(0,newScale));
        
        this.thruster(keystate);
        this.drawScale = newScale;
    },
    thruster: function(keymap)
    {
        var ax = 0;
        var ay = 0;
        var K = this.thrusterstrength;
        if (keymap.shift)
        {
        }
        else if (keymap.alt)
        {
            if (keymap.up)
                ay += K;
            if (keymap.down)
                ay -= K;
            if (keymap.right)
                ax += K;
            if (keymap.left)
                ax -= K;
        }
        else
        {
            var v = hypot(this.velpos[0],this.velpos[1]);
            if (v > 1e-6)
            {
                var ux = this.velpos[0]/v;
                var uy = this.velpos[1]/v;
                if (keymap.up)
                {
                    ax += K*ux;
                    ay += K*uy;
                }
                if (keymap.down)
                {
                    ay -= K*ux;
                    ay -= K*uy;
                }
                if (keymap.right)
                {
                    ax += K*uy;
                    ay -= K*ux;
                }
                if (keymap.left)
                {
                    ax -= K*uy;
                    ay += K*ux;
                }
            }
        }
        this.thrusterAcceleration = [ax,ay];
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
        },
        'Runge-Kutta': function(vx, f, dt)
        {
            var dvxdt1 = f(vx);
            var vx1 = weightedsum([1,dt/2], [vx,dvxdt1], 4);
            var dvxdt2 = f(vx1);
            var vx2 = weightedsum([1,dt/2], [vx,dvxdt2], 4);
            var dvxdt3 = f(vx2);
            var vx3 = weightedsum([1,dt], [vx,dvxdt3], 4);
            var dvxdt4 = f(vx3);            
            return weightedsum([1,dt/6,dt/3,dt/3,dt/6], 
                [vx,dvxdt1,dvxdt2,dvxdt3,dvxdt4], 4);
        }
    },
    getStatistics: function() {
        var result = {};
        result.radius = this.getRadius();
        result.kineticEnergy = this.getKineticEnergy();
        var mu = this.mu;
        result.potentialEnergy = -mu/result.radius;
        result.speed = Math.sqrt(2*result.kineticEnergy);
        result.angularMomentum = this.getAngularMomentum();
        var h = result.angularMomentum;
        var r = result.radius;
        var mu_e = [this.velpos[1]*h  - mu/r*this.velpos[2],
                    -this.velpos[0]*h - mu/r*this.velpos[3]];
        var e = hypot(mu_e[0],mu_e[1])/mu;
        var p = h*h/mu;
        result.eccentricity = e;
        result.minRadius = p/(1+e);
        if (e < 1)
        {
            var a = p/(1-e*e);
            result.semimajorAxis = a;
            result.maxRadius = p/(1-e);
            result.period = 2*Math.PI*Math.sqrt(a*a*a/mu);
        }
        else
        {
            result.period = NaN;
            result.semimajorAxis = NaN;
            result.maxRadius = NaN;
        }
        return result;
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
    getDrawScaleFactor: function(cw,ch)
    {
        var s = this.drawScale;
        var s2 = s >> 1;
        return Math.min(cw,ch)/2 / (1 << s2) / (s & 1 ? 1.414 : 1);
    },
    draw: function(canvas, blink)
    {
        var ctx = canvas.getContext('2d');
        var cw = canvas.width;
        var ch = canvas.height;
        var rmax = this.getDrawScaleFactor(cw,ch);
        
        var x1 = cw/2 + rmax*this.velpos[2];
        var y1 = ch/2 - rmax*this.velpos[3];
        
        ctx.fillStyle = '#000000';
        ctx.fillRect(0,0,cw,ch);
        
        ctx.fillStyle = '#ffffff';        
        // draw star
        ctx.beginPath();
        dotpath(ctx, cw/2,ch/2,5);
        ctx.fill();        

        // draw satellite
        ctx.beginPath();
        if (x1 > 0 && x1 < cw && y1 > 0 && y1 < ch)
        { 
           dotpath(ctx,x1,y1,2);
           ctx.fill();  
           ctx.beginPath()
           ctx.strokeStyle = '#00ff80';
           var vscale = rmax*0.5;
           arrowpath(ctx,x1,y1,vscale*this.velpos[0],-vscale*this.velpos[1]);
           ctx.stroke();
        }
        else
        {
            if (blink)
            {
                var marg = 5;
                var alen = 30;
                var mw = cw/2 - marg;
                var mh = ch/2 - marg;
                var x2 = x1 - cw/2;
                var y2 = y1 - ch/2;
                var ax2 = Math.abs(x2);
                var ay2 = Math.abs(y2);
                var r = hypot(x2,y2);
                var k = 1;
              
                if (ax2 > mw)
                    k = mw/ax2;
                if (ay2 > mh)
                    k = Math.min(k,mh/ay2);
                ctx.beginPath();
                ctx.strokeStyle = '#ffffff';
                var x3 = cw/2+k*x2;
                var y3 = ch/2+k*y2;
                var tx = x2/r*alen;
                var ty = y2/r*alen;
                arrowpath(ctx, x3-tx, y3-ty, tx,ty);
                ctx.stroke();
            }
        }
          
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
        var rmax = this.getDrawScaleFactor(cw,ch);

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
            /*ctx.moveTo(cx + rk*Math.cos(th_c-th_d),
                       cy - rk*Math.sin(th_c-th_d));
            ctx.lineTo(cx,cy);
            ctx.lineTo(cx + rk*Math.cos(th_c+th_d),
                       cy - rk*Math.sin(th_c+th_d));*/
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
        var N = 64;
        var e1 = Math.min(e,0.7);
        var e2 = 1-Math.sqrt(1-e1*e1);
        for (var i = 0; i <= N; ++i)
        {   
            var rho = i / N;
            // this seems to work... :/
            var drho = 0.5*e2*Math.sin(2*Math.PI*rho);
            var rhowarp = rho - drho + drho*drho - drho*drho*drho;
            var th_i = th_start + rhowarp*(th_stop-th_start);
            var xy = f(th_i);
            if (i == 0)
            {
                ctx.moveTo(xy[0],xy[1]);
            }
            else
                ctx.lineTo(xy[0],xy[1]);
            //    dotpath(ctx,xy[0],xy[1],2);
        }
    }
}

