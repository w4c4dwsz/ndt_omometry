#include <iostream>
#include <vector>
using namespace std;
int main()
{
    int R = 10;
    int C = 20;
    int count = 0;
    vector<int> v;
    for (int i = 0; i < R; i++)
    {
        v.push_back(count);
        for (int i = 0; i < C; i++)
        {
            /* code */
            // v.push_back(count);
            count++;
        }
        
    }
    for(auto a : v)
        printf("%d\n", a);
    
    printf("%d", count);
}
