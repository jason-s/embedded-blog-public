function createPendulum() {

function rateofchange(state, etc)
{
    var theta = state[0];
    var omega = state[1];
    var x = Math.sin(theta);
    var y = Math.cos(theta);
    
    var magnetcapture = 0.09;
    var incapture = x<magnetcapture && x>-magnetcapture;
    var den = x*x+magnetcapture*magnetcapture;
    var k = 0.03;
    var bottommagnet = (y > 0) ? -k*x/den/den : 0;
    var topmagnet = (y < 0) ? k*x/den/den : 0;
    var torque = -etc.g*etc.L*x + etc.force*etc.L*y + bottommagnet + topmagnet - etc.damping*omega -etc.dampingmagnet*omega*(incapture?1:0) + etc.coiltorque;

    omegadot = torque/etc.J;
    thetadot = omega; 
    return [thetadot, omegadot];
}

function eqn_timestep(state, f,dt,etc)
{
    // Runge-Kutta
    k1 = f(state,etc);
    k2 = f([state[0]+k1[0]*dt/2, state[1]+k1[1]*dt/2], etc);
    k3 = f([state[0]+k2[0]*dt/2, state[1]+k2[1]*dt/2], etc);
    k4 = f([state[0]+k3[0]*dt, state[1]+k3[1]*dt], etc);
    state[0] += (k1[0]+2*k2[0]+2*k3[0]+k4[0])*dt/6,
    state[1] += (k1[1]+2*k2[1]+2*k3[1]+k4[1])*dt/6;
}

function simulateCoils(etc, sth, which, dt)
{
    var inrange = sth < etc.coilcapture && sth > -etc.coilcapture;
    if (inrange)
    {
        if (etc.coils[which] < 0)
        {
            etc.coiltorque = which ? etc.coilstrength : -etc.coilstrength;
        }
        else
        {
            etc.coils[which] += dt;
            if (etc.coils[which] > etc.coilthreshold)
               etc.coils[which] = -1;
        }
    }
    else
    {
        if (etc.coils[which] < 0)
            etc.coils[which] = 0;
        etc.coiltorque = 0;
    }
}

function timestep(pendulum, etc, dt)
{
    etc.force = 0;
    state = [pendulum.theta, pendulum.omega];
    eqn_timestep(state, rateofchange, dt, etc);
    pendulum.theta = state[0];
    pendulum.omega = state[1];
    var sth = Math.sin(pendulum.theta);
    var cth = Math.cos(pendulum.theta);
    simulateCoils(etc, sth, cth > 0 ? 1 : 0, dt);
}

function _draw(canvas, pendulum, opt) {
    var ctx = canvas.getContext('2d');
    // Wipe canvas
    var cw = canvas.width;
    var ch = canvas.height;
    if (opt == null)
        opt = {}
    if (opt.backgrounddraw != null)
        opt.backgrounddraw(ctx,0,0,cw,ch);
    else
        ctx.clearRect(0, 0, cw, ch);
    var xc = cw/2;
    var yc = ch/2;
    var pr = Math.min(xc,yc)-2*pendulum.radius;


    var sth = Math.sin(pendulum.theta);
    var cth = Math.cos(pendulum.theta);
    var x = xc + pr * sth; 
    var y = yc + pr * cth;
    
    // Draw the box
    var opacity = opt.opacity || 1.0;
    ctx.strokeStyle = 'rgba(64,128,160,'+opacity+')';
    ctx.fillStyle = 'rgba(128,192,224,'+opacity+')';
    
    var h = 8;
    ctx.fillRect(cw*0.1,h,cw*0.8*etc.showcoils(0),h);
    ctx.strokeRect(cw*0.1,h,cw*0.8,h);
    ctx.fillRect(cw*0.1,ch-2*h,cw*0.8*etc.showcoils(1),h);
    ctx.strokeRect(cw*0.1,ch-2*h,cw*0.8,h);

    ctx.beginPath();
    ctx.arc(xc, yc, 2, 0, 2 * Math.PI, false);
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(xc,yc);
    ctx.lineTo(x,y);
    ctx.stroke();

    ctx.beginPath();    
    ctx.arc(x, y, pendulum.radius, 0, 2 * Math.PI, false);
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.stroke();
}





var pendulum = {
    theta: Math.PI*0.9146,
    omega: 0,
    radius: 20,
};

var etc = (function(){
    var etc = {};
    etc.g = 9.8;
    etc.L = 0.5;
    etc.J = etc.L*etc.L;
    etc.damping = 0.01;
    etc.dampingmagnet = 1.25;
    etc.coiltorque = 0;
    etc.coilstrength = 105;
    etc.coilcapture = 0.08;
    etc.coilthreshold = 1.3;
    etc.coils = [0,0];
    etc.showcoils = function(i) { 
        var c = this.coils[i] / this.coilthreshold;
        return c < 0 ? 0 : c > 1 ? 1 : c;
    }
    return etc;
})();

return {pendulum: pendulum,
        etc: etc,
        update: function(dt) { timestep(pendulum, etc, dt); },
        draw: function(canvas, opt) { _draw(canvas, pendulum, opt); }
       };

}
