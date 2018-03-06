#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include "math.h"
#include "string.h"
#include "std_msgs/String.h"
using namespace std;
const float bk = 0.7;
struct toa_do
{
	float x;
	float y;
};
float xet_dau (struct toa_do a, struct toa_do b, struct toa_do c) // XET DAU KHI THAY TOA DO DIEM A VAO DUONG THUONG BC
{
	float dau = ((c.y-b.y)*(a.x-b.x)-(c.x-b.x)*(a.y-b.y));
	return dau;
}
float xet_goc(struct toa_do a, struct toa_do b)
{
	float goc = atan((b.y-a.y)/(b.x-a.x));
	return goc; 
}
void xuat_du_lieu (struct toa_do a)
{	
	cout << "(" << a.x << " , " << a.y << ")" << endl;
}
int main()
{
	toa_do dinh[6];
	for (int i = 1; i <= 3; i++)
	{
			cout << "X CUA DINH " << i << ": ";
			cin >> dinh[i].x;
			cout << "Y CUA DINH " << i << ": ";
			cin >> dinh[i].y;
	}
	cout << "GOC: " << xet_goc(dinh[2],dinh[3])*180/3.14 << endl;
	cout << "DAU: " << xet_dau(dinh[1], dinh[2], dinh[3]) << endl;

	if (xet_goc(dinh[2],dinh[3]) >= 0)
	{
		dinh[4].x = dinh[1].x - bk*sin(abs(xet_goc(dinh[2],dinh[3])));
		dinh[4].y = dinh[1].y + bk*cos(abs(xet_goc(dinh[2],dinh[3])));

		dinh[5].x = dinh[1].x + bk*sin(abs(xet_goc(dinh[2],dinh[3])));
		dinh[5].y = dinh[1].y - bk*cos(abs(xet_goc(dinh[2],dinh[3])));

		if (abs(xet_dau(dinh[4],dinh[2],dinh[3]))>abs(xet_dau(dinh[5],dinh[2],dinh[3])))
		{
			dinh[6] = dinh[4];
		}
		else
		{
			dinh[6] = dinh[5];
		}
	}
	else
	{
		dinh[4].x = dinh[1].x + bk*sin(abs(xet_goc(dinh[2],dinh[3])));
		dinh[4].y = dinh[1].y + bk*cos(abs(xet_goc(dinh[2],dinh[3])));

		dinh[5].x = dinh[1].x - bk*sin(abs(xet_goc(dinh[2],dinh[3])));
		dinh[5].y = dinh[1].y - bk*cos(abs(xet_goc(dinh[2],dinh[3])));
		if (abs(xet_dau(dinh[4],dinh[2],dinh[3]))>abs(xet_dau(dinh[5],dinh[2],dinh[3])))
		{
			dinh[6] = dinh[4];
		}
		else
		{
			dinh[6] = dinh[5];
		}
	}

	cout << "DINH 6: " ; xuat_du_lieu(dinh[6]);

	return 0;
}