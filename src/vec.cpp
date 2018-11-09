class Vec{
    public:
        Vec(double x, double y, double yaw);
        ~Vec();
        double x_;
        double y_;
        double yaw_;
    private:
};

Vec::Vec(double x, double y, double yaw){
    x_ = x;
    y_ = y;
    yaw_ = yaw;
}

Vec::~Vec(){

}
