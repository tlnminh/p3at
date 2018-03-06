#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include "math.h"
#include "string.h"
#include "std_msgs/String.h"
using namespace std;
int so_vc, so_dinh;
const float bk = 0.7;
struct toa_do
{
	float x;
	float y;
	float theta;
};
struct vat_can
{
	float min_kc;
	int so_dinh;
	struct toa_do dinh[20];
	struct toa_do dinh_ngan_nhat;
	struct toa_do dinh_dai_nhat;	
	struct toa_do dinh_duong_nhat;
	struct toa_do dinh_am_nhat;
};
struct duong_di
{
	int he_so_duong_di;
	struct toa_do dinh[20];
	float do_dai;
};
float khoang_cach (struct toa_do a, struct toa_do b)
{
	float distance = sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
	return distance;
}

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
bool xet_khac_nhau (struct toa_do a, struct toa_do b)
{
	bool khac = false;
	if ((a.x != b.x) || (a.y != b.y))
	{
		khac = true;
	}
	return khac;
}
void xuat_du_lieu (struct toa_do a)
{	
	cout << "(" << a.x << " , " << a.y << ")" << endl;
}

long nhi_phan(long dec)
{
	long rem, i = 1, sum = 0;
	do
    {
        rem=dec%2;
        sum=sum + (i*rem);
        dec=dec/2;
        i=i*10;
    }while(dec>0);
    return sum;
}
bool cat_nhau(struct toa_do a, struct toa_do b, struct toa_do c, struct toa_do d)	// XEM DOAN ab CO CAT bc KHONG?
{
	bool cat = false;
	if ((xet_dau(a,c,d)*xet_dau(b,c,d) <= 0)&&(xet_dau(c,a,b)*xet_dau(d,a,b) <= 0))
	{
		cat = true;
	}
	return cat;
}
void tim_duong()
{
	// MOT SO KHAI BAO
	vat_can vat_can[20];
	toa_do muc_tieu;
	toa_do robot_first;
	toa_do robot;
	duong_di duong_di[20];
	toa_do xoa_diem;

	xoa_diem.x = 0;
	xoa_diem.y = 0;


	// NHAP THONG TIN MOI TRUONG
	cout << "SO VAT CAN: ";
	cin >> so_vc;


//	cout << "Nhi phan: " << nhi_phan(so_vc) << endl;
/*
	for (long lua_chon = 1; lua_chon <= long(pow(2,so_vc)); lua_chon++)
	{
		long bin = nhi_phan(lua_chon);
		cout << "Nhi phan: " << bin << " -> ";
		for (int i = 0; i < so_vc; i++)
		{
			int bit = (bin/long(pow(10,i)))%2;
			if (bit == 1)
			{
				cout << "Duong ";
			}
			else
			{
				cout << "Am    ";
			}
		}
		cout << endl;
	}
*/
	//NHAP TOA DO CAC VAT CAN
	for (int i = 1; i <= so_vc; i++)
	{
		cout << "SO DINH CUA VAT CAN THU " << i << ": ";
		cin >> vat_can[i].so_dinh;

		for (int j = 1; j <= vat_can[i].so_dinh; j++)
		{
			cout << "X CUA DINH " << j << " CUA VC " << i << ": ";
			cin >> vat_can[i].dinh[j].x;
			cout << "Y CUA DINH " << j << " CUA VC " << i << ": ";
			cin >> vat_can[i].dinh[j].y;

			xuat_du_lieu(vat_can[i].dinh[j]);
		}
	}
/*
	// XUAT DU LIEU TOA DO CAC VAT CAN
	for (int i = 1; i <= so_vc; i++)
	{
		cout << "VC " << i << " : " << endl;

		for (int j = 1; j <= vat_can[i].so_dinh; j++)
		{
			xuat_du_lieu(vat_can[i].dinh[j]);
		}
	}
*/
	// NHAP DU LIEU TARGET
	cout << "X MUC TIEU: ";
	cin >> muc_tieu.x;
	cout << "Y MUC TIEU: ";
	cin >> muc_tieu.y;

	// NHAP TU THE ROBOT
	cout << "X ROBOT: ";
	cin >> robot_first.x;
	cout << "Y ROBOT: ";
	cin >> robot_first.y;

//	cout << "GOC ROBOT: ";
//	cin >> robot_first.theta;


	
//	cout << muc_tieu.x << "   " << muc_tieu.y << endl;
//	cout << robot.x << "   " << robot.y << "   " << robot.theta << endl;


	// BAT DAU THUC HIEN


for (int lua_chon = 1; lua_chon <= pow(2,so_vc); lua_chon++)
{
	bool cat = false;
	bool cat_2 = false;
	int vc_hien_tai = 0;
	int lan = 1;

	duong_di[lua_chon].dinh[0] = robot_first;
	duong_di[lua_chon].he_so_duong_di = 1;
	robot = robot_first;

	cout << "--------------- LUA CHON: " << lua_chon << " ----------------------" << endl;
	while (xet_khac_nhau(robot,muc_tieu) == true)	// NEU ROBOT CHUA DEN MUC TIEU THI TIEP TUC LAP
	{
		//cout << "LAN: " << lan << endl; lan++;
		for (int i = 1; i <= so_vc; i++)
		{
			//cout << "FOR 1" << endl;
			vat_can[i].min_kc = -1;		// NEU VAT CAT THI min_kc SE THAY DOI
			for (int j = 2; j <= vat_can[i].so_dinh; j++)
			{
				//cout << "FOR 2" << endl;
				// XEM VAT CO CAT KHONG?
				if ((xet_dau(vat_can[i].dinh[1], robot, muc_tieu)*xet_dau(vat_can[i].dinh[j], robot, muc_tieu) < 0) && (khoang_cach(vat_can[i].dinh[j],muc_tieu) < khoang_cach(robot,muc_tieu)))
				// XET DINH DA CAT DUONG THANG NAY NHUNG PHAI GAN DEN MUC TIEU HON LA ROBOT  
				{
					//cout << "IF 1" << endl;
					// VAT THE NAY DA CAT DUONG THANG
					j = vat_can[i].so_dinh + 1;
					//cout << "VAT CAN " << i << " CAT" << endl;
					cat = true;
					cat_2 = true;
				} 			
			}
			// NEU VAT CAT, TIM KHOANG CACH TUONG DOI TU VAT DEN ROBOT
			if (cat == true) 
			{
				//cout << "IF 2" << endl;
				vat_can[i].min_kc = khoang_cach(vat_can[i].dinh[1], robot);
				for (int j = 2; j <= vat_can[i].so_dinh; j++)
				{
					if (khoang_cach(vat_can[i].dinh[j], robot) < vat_can[i].min_kc)
					{
						vat_can[i].min_kc = khoang_cach(vat_can[i].dinh[j], robot);
					}
				}
				//cout << "kc: " << vat_can[i].min_kc << endl;
				cat = false;
			}
			else
			{
				//cout << "VAT CAN " << i << " KHONG CAT" << endl;
			}

		}
	if (cat_2 == true)
	{
		// TIM VAT CAT GAN NHAT TRONG SO NHUNG VAT CAT
		float min_vc = 10000;
		int min_vc_hs = 0; // HE SO VAT CAT GAN NHAT
		for (int i = 1; i <= so_vc; ++i)
		{
			if ((vat_can[i].min_kc > 0) && (vat_can[i].min_kc < min_vc))
			{
				min_vc = vat_can[i].min_kc;
				min_vc_hs = i;
			}
		}
		//cout << "VAT CAT GAN NHAT: " << min_vc_hs << endl;
	

		// TIM DINH SAO CHO DUONG DI GAN NHAT, LAM VIEC VOI VAT CAT min_vc_hs
		vat_can[min_vc_hs].dinh_am_nhat = vat_can[min_vc_hs].dinh[1];
		vat_can[min_vc_hs].dinh_duong_nhat = vat_can[min_vc_hs].dinh[1];
		for (int j = 2; j <= vat_can[min_vc_hs].so_dinh; j++)
		{
			float bien_tam = xet_dau(vat_can[min_vc_hs].dinh[j], robot, muc_tieu);
			// cout << "bien_tam: " << bien_tam << endl;
			if (bien_tam > xet_dau(vat_can[min_vc_hs].dinh_duong_nhat, robot, muc_tieu))
			{
				vat_can[min_vc_hs].dinh_duong_nhat = vat_can[min_vc_hs].dinh[j];
			}
			if (bien_tam < xet_dau(vat_can[min_vc_hs].dinh_am_nhat, robot, muc_tieu))
			{
				vat_can[min_vc_hs].dinh_am_nhat = vat_can[min_vc_hs].dinh[j];
			}
		}

		//cout << "DINH DUONG NHAT:  "; xuat_du_lieu(vat_can[min_vc_hs].dinh_duong_nhat);
		//cout << "DINH AM NHAT:  " ; xuat_du_lieu(vat_can[min_vc_hs].dinh_am_nhat);

		long bin = nhi_phan(lua_chon);
		//cout << "SO NHI PHAN: " << bin  << endl;
		int bit = (bin/long(pow(10,min_vc_hs-1)))%2;
		//cout << "BIT: " << bit << endl;
		if (bit == 1)
		{
			//cout << "CHON: DUONG " << endl;
			vat_can[min_vc_hs].dinh_ngan_nhat = vat_can[min_vc_hs].dinh_duong_nhat;
			vat_can[min_vc_hs].dinh_dai_nhat = vat_can[min_vc_hs].dinh_am_nhat;
		}
		else
		{
			//cout << "CHON: AM    " << endl;
			vat_can[min_vc_hs].dinh_ngan_nhat = vat_can[min_vc_hs].dinh_am_nhat;
			vat_can[min_vc_hs].dinh_dai_nhat = vat_can[min_vc_hs].dinh_duong_nhat;
		}

/*
		float kc_duong = khoang_cach(vat_can[min_vc_hs].dinh_duong_nhat,robot) + khoang_cach(vat_can[min_vc_hs].dinh_duong_nhat,muc_tieu);
		float kc_am = khoang_cach(vat_can[min_vc_hs].dinh_am_nhat,robot) + khoang_cach(vat_can[min_vc_hs].dinh_am_nhat, muc_tieu);
		if (kc_duong < kc_am)		
		{
			vat_can[min_vc_hs].dinh_ngan_nhat = vat_can[min_vc_hs].dinh_duong_nhat;
			vat_can[min_vc_hs].dinh_dai_nhat = vat_can[min_vc_hs].dinh_am_nhat;
		}
		else if (kc_duong > kc_am)	
		{
			vat_can[min_vc_hs].dinh_ngan_nhat = vat_can[min_vc_hs].dinh_am_nhat;
			vat_can[min_vc_hs].dinh_dai_nhat = vat_can[min_vc_hs].dinh_duong_nhat;
		}
		else 
		{
			cout << "HAI DINH NGAN NHU NHAU" << endl;
			if (khoang_cach(vat_can[min_vc_hs].dinh_duong_nhat,robot) < khoang_cach(vat_can[min_vc_hs].dinh_am_nhat,robot))
			{
				vat_can[min_vc_hs].dinh_ngan_nhat = vat_can[min_vc_hs].dinh_duong_nhat;
				vat_can[min_vc_hs].dinh_dai_nhat = vat_can[min_vc_hs].dinh_am_nhat;
			}
			else
			{
				vat_can[min_vc_hs].dinh_ngan_nhat = vat_can[min_vc_hs].dinh_am_nhat;
				vat_can[min_vc_hs].dinh_dai_nhat = vat_can[min_vc_hs].dinh_duong_nhat;
			}
		}
*/
		//cout << "DINH NGAN NHAT:  "; xuat_du_lieu(vat_can[min_vc_hs].dinh_ngan_nhat);

		robot = vat_can[min_vc_hs].dinh_ngan_nhat;
		duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di] = robot;
		duong_di[lua_chon].he_so_duong_di ++;


		// XET VAT CAT GAN NHAT XEM TREN DUONG DI CON CAT DINH NAO KHAC KO?
		for (int j = 1; j <= vat_can[min_vc_hs].so_dinh; j++)
		{
			if (xet_khac_nhau(vat_can[min_vc_hs].dinh[j], robot) == true)
			{
				
				// LOAI DIEM CAT GAN PHIA MUC TIEU
				if (xet_dau(vat_can[min_vc_hs].dinh[j],robot,muc_tieu)*xet_dau(vat_can[min_vc_hs].dinh_dai_nhat,robot,muc_tieu) < 0)
				{
					robot = vat_can[min_vc_hs].dinh[j];
					duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di] = robot;
					duong_di[lua_chon].he_so_duong_di ++;
				}
				
				// LOAI DIEM CAT GAN PHIA START
				if (duong_di[lua_chon].he_so_duong_di >= 2)
				{
					if (xet_dau(vat_can[min_vc_hs].dinh[j],duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di-1],duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di-3])*xet_dau(muc_tieu,duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di-1],duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di-3]) < 0)
					{
						duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di] = robot;
						duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di-1] = vat_can[min_vc_hs].dinh[j];
						duong_di[lua_chon].he_so_duong_di ++;
					}
				}
			}
		}

		cat_2 = false;
	}
	else //	NEU KHONG CAT VAT NAO HET
	{
		//cout << "KHONG CAT VAT NAO HET, DI THANG VE MUC TIEU " << endl;
		robot = muc_tieu;
		duong_di[lua_chon].dinh[duong_di[lua_chon].he_so_duong_di] = robot;
		duong_di[lua_chon].he_so_duong_di ++;
	}
	} // HET WHILE

	// TRUOC KHI LOAI DINH THUT
	cout << "DUONG DI " << lua_chon << " : " << endl;
	for (int h = 0; h < duong_di[lua_chon].he_so_duong_di; ++h)
	{
				xuat_du_lieu(duong_di[lua_chon].dinh[h]);
	}
/*
	// XET TREN DUONG DI CO DIEM NAO BI THUT VAO KHONG? NHUNG CHI XET TRUONG HOP CO 3 DIEM DUONG DI he_so_duong_di - 1 > 3 
	if (duong_di[lua_chon].he_so_duong_di > 4)
	{
		for (int h = 2; h < duong_di[lua_chon].he_so_duong_di - 1; h++) // CHON DAU < THAY VI <= DO CUOI CUNG he_so_duong_di ++
		{
			if (xet_dau(duong_di[lua_chon].dinh[h], duong_di[lua_chon].dinh[h-1], duong_di[lua_chon].dinh[h+1])*xet_dau(muc_tieu, duong_di[lua_chon].dinh[h-1], duong_di[lua_chon].dinh[h+1]) > 0)
			{
				int vc; // VC LA VAT CAN CHUA HAI DINH h-1 VA h
				bool cat_3 = false; // NEU DUONG h-1 DEN h+1 CAT VAT
				for (int i = 1; i <= so_vc; ++i)	// XET TAT CA CAC VAT
				{
					for (int j = 1; j < vat_can[i].so_dinh; j++) // XET TAT CA CAC DINH CUA VAT DO
					{
						// TRUONG HOP h-1 VA h CUNG THUOC MOT VAT
						if (xet_khac_nhau(duong_di[lua_chon].dinh[h-1],vat_can[i].dinh[j]) == false) // NEU DINH TRUNG VOI MOT DINH NAO DO CUA VC
						{
							for (int m = 1; m <= vat_can[i].so_dinh; m++) // XET TAT CA CAC DINH CUA VAT DO MOT LAN NUA
							{
								// NEU DINH CUA VC KHAC VOI DINH h va h-1 CUA DUONG DI
								if ((xet_khac_nhau(vat_can[i].dinh[m], duong_di[lua_chon].dinh[h-1]) == true) && (xet_khac_nhau(vat_can[i].dinh[m], duong_di[lua_chon].dinh[h]) == true))
								{
									if (xet_dau(duong_di[lua_chon].dinh[h], duong_di[lua_chon].dinh[h-1], duong_di[lua_chon].dinh[h+1])*xet_dau(vat_can[i].dinh[m], duong_di[lua_chon].dinh[h-1], duong_di[lua_chon].dinh[h+1]) < 0)
									{
										cat_3 = true;
										m = vat_can[i].so_dinh + 1;
										j = vat_can[i].so_dinh;
										i = so_vc + 1;
									}
								}
							}
						}
						if (xet_khac_nhau(duong_di[lua_chon].dinh[h],vat_can[i].dinh[j]) == false) // NEU DINH TRUNG VOI MOT DINH NAO DO CUA VC
						{
							for (int m = 1; m <= vat_can[i].so_dinh; m++) // XET TAT CA CAC DINH CUA VAT DO MOT LAN NUA
							{
								// NEU DINH CUA VC KHAC VOI DINH h va h-1 CUA DUONG DI
								if (xet_khac_nhau(vat_can[i].dinh[m], duong_di[lua_chon].dinh[h]) == true)
								{
									if (xet_dau(duong_di[lua_chon].dinh[h], duong_di[lua_chon].dinh[h-1], duong_di[lua_chon].dinh[h+1])*xet_dau(vat_can[i].dinh[m], duong_di[lua_chon].dinh[h-1], duong_di[lua_chon].dinh[h+1]) < 0)
									{
										cat_3 = true;
										m = vat_can[i].so_dinh + 1;
										j = vat_can[i].so_dinh;
										i = so_vc + 1;
									}
								}
							}
						}
					}
				}
				if (cat_3 == false) // NEU KHONG CAT THI XOA DIEM THUT
				{
					for (int n = h; n <= duong_di[lua_chon].he_so_duong_di - 1; n++)
					{
						duong_di[lua_chon].dinh[n] = duong_di[lua_chon].dinh[n+1];
					}
					duong_di[lua_chon].he_so_duong_di--;
				}
				
			}	
		}
	}
*/
	for (int h = 2; h < duong_di[lua_chon].he_so_duong_di - 1; h++)		// XET CAC DINH CUA DUONG DI
	{
		bool cat_4 = false;
		for (int i = 1; i <= so_vc; i++)	// XET TAT CA CAC VAT 
		{
			for (int dinh_1 = 1; dinh_1 <= vat_can[i].so_dinh; dinh_1++)	// CHON 1 DINH CUA VAT CAN
			{
				if ((xet_khac_nhau(vat_can[i].dinh[dinh_1],duong_di[lua_chon].dinh[h-1])==true)&&(xet_khac_nhau(vat_can[i].dinh[dinh_1],duong_di[lua_chon].dinh[h+1])==true))
				{
					for (int dinh_2 = 1; dinh_2 <= vat_can[i].so_dinh; dinh_2++)	// CHON 1 DINH KHAC CUA VAT CAN
					{
						// GIO DA CO DOAN THANG dinh_1-dinh_2 VA DOAN THANG h-1 - h+1
						if ((xet_khac_nhau(vat_can[i].dinh[dinh_2],duong_di[lua_chon].dinh[h-1])==true)&&(xet_khac_nhau(vat_can[i].dinh[dinh_2],duong_di[lua_chon].dinh[h+1])==true))
						{
							if (dinh_1 != dinh_2)
							{
								if (cat_nhau(duong_di[lua_chon].dinh[h-1], duong_di[lua_chon].dinh[h+1], vat_can[i].dinh[dinh_1],vat_can[i].dinh[dinh_2]))
								{
									cat_4 = true;
									dinh_2 = vat_can[i].so_dinh+1;
									dinh_1 = vat_can[i].so_dinh+1;
									i = vat_can[i].so_dinh+1;
								}
							}
						}
					}	
				}
			}
		}
		if (cat_4 == false) // NEU KHONG CAT THI XOA DIEM THUT
		{
			for (int n = h; n <= duong_di[lua_chon].he_so_duong_di - 1; n++)
			{
				duong_di[lua_chon].dinh[n] = duong_di[lua_chon].dinh[n+1];
			}
			duong_di[lua_chon].he_so_duong_di--;
		}
	}

	// CONG THEM MOT KHOANG BANG BAN KINH
	int hs = duong_di[lua_chon].he_so_duong_di;
	for (int h = 1; h <= duong_di[lua_chon].he_so_duong_di - 2; h++)
	{
		float goc = abs(xet_goc(duong_di[lua_chon].dinh[h-1],duong_di[lua_chon].dinh[h+1]));
		if (goc >= 0)
		{
			duong_di[lua_chon].dinh[hs].x = duong_di[lua_chon].dinh[h].x - bk*sin(goc);
			duong_di[lua_chon].dinh[hs].y = duong_di[lua_chon].dinh[h].y + bk*cos(goc);

			duong_di[lua_chon].dinh[hs+1].x = duong_di[lua_chon].dinh[h].x + bk*sin(goc);
			duong_di[lua_chon].dinh[hs+1].y = duong_di[lua_chon].dinh[h].y - bk*cos(goc);

			float xet_dau_hs = abs(xet_dau(duong_di[lua_chon].dinh[hs],duong_di[lua_chon].dinh[h-1],duong_di[lua_chon].dinh[h+1]));
			float xet_dau_hs1 = abs(xet_dau(duong_di[lua_chon].dinh[hs+1],duong_di[lua_chon].dinh[h-1],duong_di[lua_chon].dinh[h+1]));
			if (xet_dau_hs > xet_dau_hs1)
			{
				duong_di[lua_chon].dinh[h] = duong_di[lua_chon].dinh[hs];
			}
			else
			{
				duong_di[lua_chon].dinh[h] = duong_di[lua_chon].dinh[hs+1];
			}
		}
		else
		{
			duong_di[lua_chon].dinh[hs].x = duong_di[lua_chon].dinh[h].x + bk*sin(goc);
			duong_di[lua_chon].dinh[hs].y = duong_di[lua_chon].dinh[h].y + bk*cos(goc);

			duong_di[lua_chon].dinh[hs+1].x = duong_di[lua_chon].dinh[h].x - bk*sin(goc);
			duong_di[lua_chon].dinh[hs+1].y = duong_di[lua_chon].dinh[h].y - bk*cos(goc);
			float xet_dau_hs = abs(xet_dau(duong_di[lua_chon].dinh[hs],duong_di[lua_chon].dinh[h-1],duong_di[lua_chon].dinh[h+1]));
			float xet_dau_hs1 = abs(xet_dau(duong_di[lua_chon].dinh[hs+1],duong_di[lua_chon].dinh[h-1],duong_di[lua_chon].dinh[h+1]));
			if (xet_dau_hs > xet_dau_hs1)
			{
				duong_di[lua_chon].dinh[h] = duong_di[lua_chon].dinh[hs];
			}
			else
			{
				duong_di[lua_chon].dinh[h] = duong_di[lua_chon].dinh[hs+1];
			}
		}
	}


	// XONG TIM DUONG, XUAT THU TU DUONG DI
	cout << "DUONG DI SAU KHI LOAI DINH THUT VA THEM BAN KINH: " << endl;
	for (int h = 0; h < duong_di[lua_chon].he_so_duong_di; ++h)
	{	/*
		if ((h == 0)|| ((h != 0)&&(xet_khac_nhau(duong_di[lua_chon].dinh[h], xoa_diem) == true)))
		{
			xuat_du_lieu(duong_di[lua_chon].dinh[h]);
		}
		*/
		xuat_du_lieu(duong_di[lua_chon].dinh[h]);
	}
} // HET MOT LUA CHON
/*
	// CONG THEM BAN KINH CHO DUONG DI
	for (int lua_chon = 1; lua_chon <= pow(2,so_vc); lua_chon++)
	{
		for (int h = 1; h <= duong_di[lua_chon].he_so_duong_di - 2; h++) // CHON DAU < THAY VI <= DO CUOI CUNG he_so_duong_di ++
		{
			if ((xet_khac_nhau(duong_di[lua_chon].dinh[h],duong_di[lua_chon].dinh[h-1]) == true )&&(xet_khac_nhau(duong_di[lua_chon].dinh[h],duong_di[lua_chon].dinh[h+1]) == true))
		}
	}
*/

	// TINH DUONG DI CUA TAT CA CAC DUONG
	cout << "--------------------------------" << endl;
	cout << "DO DAI DUONG DI: " << endl;
	for (long lua_chon = 1; lua_chon <= long(pow(2,so_vc)); lua_chon++)
	{
		for (int h = 0; h < duong_di[lua_chon].he_so_duong_di-1; ++h)
		{
			duong_di[lua_chon].do_dai += khoang_cach(duong_di[lua_chon].dinh[h],duong_di[lua_chon].dinh[h+1]);
		} 
		cout << "THU " << lua_chon << " LA: " << duong_di[lua_chon].do_dai << endl;
	}

	int ngan_nhat = 1;
	for (long lua_chon = 2; lua_chon <= long(pow(2,so_vc)); lua_chon++)
	{
		if (duong_di[lua_chon].do_dai < duong_di[ngan_nhat].do_dai)
		{
			ngan_nhat = lua_chon;
		}
	}
	cout << "-------------------------KET QUA--------------------------------" << endl;
	cout << "DE XUAT DUONG DI SO: " << ngan_nhat << " VOI DO DAI: " << duong_di[ngan_nhat].do_dai 
	<< " GOM CAC DIEM: " << endl;
	for (int h = 0; h < duong_di[ngan_nhat].he_so_duong_di; ++h)
	{
		xuat_du_lieu(duong_di[ngan_nhat].dinh[h]);
	}

	// BAT DAU CHO XE CHAY THEO DUONG DI
	char ki_tu;
	cout << "NHAN S DE BAT DAU DI, HOAC NHAN PHIM BAT KI DE THOAT" << endl;
	cin >> ki_tu;
	if ((ki_tu == 's')||(ki_tu == 'S'))
	{
		cout << "asdasd" << endl;
	}
}
int main()
{
	tim_duong();

	return 0;
}

