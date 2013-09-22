function Gravity1Simulation()
{
    this.M1G = 0.5;
    this.minR = 0.05;
    this.velpos = [0,0.7,0.5,0];
    this.solver = 'Trapezoidal';
    this.initialEnergy = this.getKineticEnergy() + this.getPotentialEnergy();
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

function dot(ctx,x,y,r)
{
    ctx.beginPath();
    ctx.arc(x, y, r, 0, 2 * Math.PI, false);
}
function sumsq(x,y) { return x*x+y*y; }
function drawbar(ctx,x0,dx,height,hmargin,e)
{
    var dy = e*(height-2*hmargin);
    ctx.fillRect(x0,height-hmargin-dy,dx,dy);
}

Gravity1Simulation.prototype = 
{
    // 4-vector containing velx, vely, posx, posy
    calcDerivative: function(vx)
    {
        var r = Math.sqrt(vx[2]*vx[2] + vx[3]*vx[3]);
        // |a| = F/M2 = 1/M2*(M1*M2*G/r/r) = M1*G/r/r
        // a = -|a|*x/|x| = -|a|*x/r
        var k = -this.M1G/r/r/r;
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
    getKineticEnergy: function() { return sumsq(this.velpos[0],this.velpos[1])/2; },
    getPotentialEnergy: function() {
        var rsquared=sumsq(this.velpos[2],this.velpos[3]);
        return this.M1G*(1.0/this.minR - 1.0/Math.sqrt(rsquared));
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
        dot(ctx, cw/2,ch/2,5);
        ctx.fill();        

        dot(ctx,x1,y1,2);
        ctx.fill();  
        
        var e0=0.8 / this.initialEnergy;
        var ke=e0*this.getKineticEnergy();
        var pe=e0*this.getPotentialEnergy();
        
        ctx.fillStyle = '#ff8080';
        drawbar(ctx,cw-13,2,ch,2,ke);        
        ctx.fillStyle = '#80ff80';
        drawbar(ctx,cw-9,2,ch,2,pe);   
        ctx.fillStyle = '#8080ff';
        drawbar(ctx,cw-5,2,ch,2,ke+pe);     
    }   
}

