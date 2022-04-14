class RangeSensor
{
public:
        RangeSensor();

        float getDistance();
private:
        float smoothDistance_;
        float smoothe(const float input, const float data);
};