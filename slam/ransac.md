ransac找直线 opencv

```c++
void fitLineRansac(const vector<cv::Point2d>& points,
                   cv::Vec4f &line,
                   int iterations = 1000,
                   double sigma = 1.0,
                   double k_min = -7,
                   double k_max = 7)
{
    int n = points.size();
    if(n < 2){
        return;
    }

    cv::RNG rng;
    double bestScore = -1;
    for(int k = 0; k < iterations; k++){
        int i1 = 0, i2 = 0;
        while(i1 == i2){
            i1 = rng(n);
            i2 = rng(n);
        }
        const cv::Point2f& p1 = points[i1];
        const cv::Point2f& p2 = points[i2];

        cv::Point2f dp = p1 - p2;
        dp = dp * 1./norm(dp);
        double score = 0;

        if(dp.y/dp.x <= k_max && dp.y/dp.x >= k_min){
            for(int i = 0; i < n; i++){
                cv::Point2f v = points[i] - p1;
                double d = v.y * dp.x - v.x * dp.y; //向量a与b叉乘/向量b的摸.||b||=1./norm(dp)
                //score += exp(-0.5*d*d/(sigma*sigma));//误差定义方式的一种
                if(fabs(d) < sigma){
                    score += 1;
                }
            }
        }
        if(score > bestScore){
            bestScore = score;
            line = cv::Vec4f(dp.x, dp.y, p1.x, p1.y);
        }
    }
}
```

ransac找直线 Eigen

```c++
void fitLineRansac(const vector<Vector2d>& points,
                   Vector4d &line,
                   int iterations = 1000,
                   double sigma = 1.0,
                   double k_min = -7,
                   double k_max = 7)
{
    int n = points.size();
    if(n < 2){
        return;
    }

    double bestScore = -1;
    for(int k = 0; k < iterations; k++){
        int i1 = 0, i2 = 0;
        while(i1 == i2){
            i1 = rand() % n;
            i2 = rand() % n;
        }
        const Vector2d& p1 = points[i1];
        const Vector2d& p2 = points[i2];

        Vector2d dp = p1 - p2;
        dp = dp * 1./dp.norm();
        double score = 0;

        if(dp(1)/dp(0) <= k_max && dp(1)/dp(0) >= k_min){
            for(int i = 0; i < n; i++){
                Vector2d v = points[i] - p1;
                double d = v(1) * dp(0) - v(0) * dp(1); //向量a与b叉乘/向量b的摸.||b||=1./norm(dp)
                //score += exp(-0.5*d*d/(sigma*sigma));//误差定义方式的一种
                if(fabs(d) < sigma){
                    score += 1;
                }
            }
        }
        if(score > bestScore){
            bestScore = score;
            line = Vector4d (dp(0), dp(1),p1(0), p1(1));
        }
    }
}
```