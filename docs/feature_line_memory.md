# Bo nho duong di + ne vat can roi nhap lai line — TRANG THAI TRIEN KHAI (2026-11-24)

Yeu cau goc: chu du an dua (prompt "BAM LINE CO BO NHO DUONG DI + NE VAT CAN ROI NHAP LAI LINE").
Thich nghi voi xe that: STM32 gui encoder 4 banh + goc gyro (Pi tu tinh odometry), anh 640x480, san go,
dung lai `ground_calib.json` (hang anh -> khoang cach, sai so 2 mm; ti le mm/pixel ngang) thay homography giay luoi.

## File
| File | Noi dung |
|---|---|
| `pi/memory_nav.py` | Odometry (noi suy theo thoi diem chup), LineMemory (polyline 2 cm, gop/lam muot, goc vuong giu nhon, noi cau sau khoang trong, chong troi P4, luu/nap tuyen P5), DetourPlan (smoothstep bac 5), plan_detour, pure pursuit, BANG CHUYEN TRANG THAI, ve PNG |
| `pi/line_vision.py` | them `g_pts`: diem tam line tren san (mm, khung xe) |
| `pi/line_follow.py` | che do MEMORY / DEXEC / REJOIN, phat hien vat TREN DUONG cach mui <= 70 cm, log `logs/pose_N.csv`, `logs/mem_N.png/json`, `logs/drift_N.csv`, `--route <ten>` |
| `pi/sim_world.py` | MO PHONG chay nguyen line_follow khong can xe (camera/LiDAR/STM32 gia) |
| `pi/tests/test_memory_nav.py` | 17 test (chay ca tren Pi) |
| `pi/config.yaml` muc `memory:` | tat ca tham so (mm, mm/s) |

## Trang thai
FOLLOW -> MEMORY -> FOLLOW (mat line tam thoi, toi da 60 cm) | FOLLOW -> BLOCKED -> (DPLAN) -> DEXEC -> REJOIN -> FOLLOW |
DPLAN that bai -> BLOCKED (cho) | REJOIN khong thay line sau 40 cm -> ERROR | moi trang thai co timeout.
Line gay dai hon tam nhin camera: trong giong cuoi line -> dang chay them ma thay line lai phia truoc -> bam tiep.

## Ket qua mo phong (sim_world.py)
| Kich ban | Ket qua |
|---|---|
| chai tren doan thang (thay tu 66 cm) | ne trai 35 cm, nhap lai lech 3 cm, toi cuoi line, 0 va cham |
| vat roi dot ngot cach mui 13 cm | truot ngang truoc roi ne, toi cuoi line, 0 va cham |
| chai truoc doan cong | nhap lai lech 11 cm, can giua, toi cuoi line |
| line gay 40 cm | chay qua, bam tiep |
| loa camera 40 cm | bam bo nho 40 cm roi thay lai line |
| chai sat GOC VUONG | khong co tuyen: ne theo ngoai suy, khong thay line -> dung bao loi (an toan); co tuyen: tu choi ne "vat gan goc re" |

## Gioi han da biet
- Quy dao lech song song khong di vong goc vuong -> vat trong khoang ~1 m quanh goc: dung cho.
- Camera khong dung de chan khi dang ne (LiDAR + vat da ghi lai la lop an toan) -> vat thap mau den xuat hien NGAY trong luc ne co the khong thay.
- Chua do tren xe that: sai so tai tao duong, do troi truoc/sau hieu chinh, ti le ne thanh cong, sai lech nhap lai (>= 10 lan).
