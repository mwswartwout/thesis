upper.half.circle <- function(x,y,r,nsteps=100,...){  
  rs <- seq(0,pi,len=nsteps)
  circle <- data.frame(xc = x+r*cos(rs), yc = y+r*sin(rs))
  return(circle)
} 

lower.half.circle <- function(x,y,r,nsteps=100,...){ 
  rs <- seq(0,pi,len=nsteps) 
  xc <- x-r*cos(rs) 
  yc <- y-r*sin(rs) 
  polygon(xc,yc,...) 
}

plot(1, type="n",xlab="", ylab="",xlim=c(-2,2),ylim=c(-2,2), asp=1)
points <- upper.half.circle(0,0,1,nsteps=1000)
