
constexpr int max_nr_callback = 4;
using callback_t = void (*)();
void reg_callback(callback_t){};

class A
{
private:
    static A *registry[max_nr_callback];
    template <int idx>
    static void static_callback()
    {
        registry[idx]->callback();
    }
    constexpr static callback_t callback_pool[max_nr_callback] =
        {
            static_callback<0>,
            static_callback<1>,
            static_callback<2>,
            static_callback<3>,
    };
    static int nr_used_callbacks;
    void callback();

public:
    A()
    {
        auto idx = nr_used_callbacks++;
        reg_callback(callback_pool[idx]);
        reg_callback(callback_pool[idx]);
        registry[idx] = this;
    }
};
int main()
{
    A a();
}