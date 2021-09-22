package jp.jaxa.iss.kibo.rpc.vietnam;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Rect;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import static org.opencv.android.Utils.matToBitmap;

public class YourService extends KiboRpcService
{
	int Pattern = 0;
	boolean QRCodeFinish = false,
			ARCodeFinish = false;

	final Quaternion Quaternion_A 	= new Quaternion(0, 0, -0.707f, 0.707f);
	Quaternion Quaternion_Target 	= new Quaternion(0, 0, -0.707f, 0.707f);
	final Point Point_A 	= new Point(11.21, -9.8, 4.79);
	final Point Point_B 	= new Point(10.6, -8.0, 4.5);
	final Point Point_AB 	= new Point(10.49, -8.8, 4.645);
	Point Point_A_Prime 	= new Point(0, 0, 0);
	Point Point_Target 		= new Point(11.219, -10.585, 5.603);

	protected void runPlan1()
	{
		Log.i("Robot[02-State]: ", "Starting the mission");
		api.startMission();
		Log.i("Robot[03-State]: ", "Laser is on");
		api.laserControl(true);
		Log.i("Robot[04-State]: ", "QR Code scanning");
		readQR();
		Log.i("Robot[05-State]: ", "Get the target point");
		getTargetPosition();
		Log.i("Robot[06-State]: ", "Aim to target point");
		moveTo(Point_A, Quaternion_Target);
		Log.i("Robot[07-State]: ", "Take a snapshot");
		api.takeSnapshot();
		api.laserControl(false);
		Log.i("Robot[08-State]: ", "Move to A-B point");
		moveTo(Point_AB, Quaternion_A);
		Log.i("Robot[09-State]: ", "Move to B point");
		moveTo(Point_B, Quaternion_A);
		Log.i("Robot[10-State]: ", "Mission completed");
		api.reportMissionCompletion();
	}
	protected void runPlan2() { }
	protected void runPlan3() { }
	public double[] getVector(double[] p,double[] q)
	{
		return new double[]{p[0]-q[0], p[1]-q[1], p[2]-q[2]};
	}
	public double[] getRoot(double a ,double b , double c)
	{
		double InSqrt = Math.pow(b, 2)-4*a*c;
		return new double[]
				{
						(-b+Math.sqrt(InSqrt))/2*a,
						(-b-Math.sqrt(InSqrt))/2*a
				};
	}
	public double getVal(double[] p)
	{
		return Math.sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]));
	}
	public double[] getQua(double[] p,double[] q)
	{
		double [] n = vecCross(p,q);
		double [] r = {p[0]-q[0],p[1]-q[1],p[2]-q[2]};
		double [] v =
				{
						Math.sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2])),
						Math.sqrt((q[0]*q[0])+(q[1]*q[1])+(q[2]*q[2])),
						Math.sqrt((r[0]*r[0])+(r[1]*r[1])+(r[2]*r[2])),
						Math.sqrt((n[0]*n[0])+(n[1]*n[1])+(n[2]*n[2]))
				};
		double [] u = {n[0]/v[3], n[1]/v[3], n[2]/v[3]};
		double theta =  Math.acos(((v[0]*v[0])+(v[1]*v[1])-(v[2]*v[2]))/(2*v[0]*v[1]));
		return new double[]
				{
						Math.cos(theta/2),
						Math.sin(theta/2)*u[0],
						Math.sin(theta/2)*u[1],
						Math.sin(theta/2)*u[2]
				};
	}
	public double[] vecCross(double[] p,double[] q)
	{
		return new double[]
				{
						p[1]*q[2]-p[2]*q[1],
						p[2]*q[0]-p[0]*q[2],
						p[0]*q[1]-p[1]*q[0]
				};
	}
	public void moveTo(Point point, Quaternion quaternion)
	{
		Result result;
		int count = 0, max_count = 2;
		do
		{
			result = api.moveTo(point, quaternion, true);
			count++;
		}
		while (!result.hasSucceeded() && count < max_count);
	}
	public double[] StringParseToDouble(String contents)
	{
		Log.d("StringParseToDouble[State]: ", "Starting");
		Log.d("StringParseToDouble[Input]: ", "" + contents);
		double pattern = 0, point_x = 0, point_y = 0, point_z = 0;
		try
		{
			String contents_cut = contents.substring(1, 35);      
			String[] commaParse = contents_cut.split(",");  
			String[] pParse = commaParse[0].split(":");     
			String[] xParse = commaParse[1].split(":");     
			String[] yParse = commaParse[2].split(":");     
			String[] zParse = commaParse[3].split(":");
			pattern = Double.parseDouble(pParse[1]);
			point_x = Double.parseDouble(xParse[1]);
			point_y = Double.parseDouble(yParse[1]);
			point_z = Double.parseDouble(zParse[1]);
			Log.d("StringParseToDouble[Info]: ", "p: " + pattern + ", x: " + point_x + ", y: " + point_y + ", z: " + point_z);
		}
		catch (Exception error)
		{
			Log.e("StringParseToDouble[State]: ", "Failure");
			Log.e("StringParseToDouble[Error]: ", error.getMessage());
		}
		Log.d("StringParseToDouble[State]: ", "Finished");
		return new double[]{pattern, point_x, point_y, point_z};
	}
	public Rect CustomCrop(int originWidth, int originHeight,  int offsetTop, int offsetBottom, int offsetLeft, int offsetRight)
	{
		Log.d("CustomCrop[State]: ", "Starting");
		int width	= originWidth-offsetLeft-offsetRight;
		int height 	= originHeight-offsetTop-offsetBottom;	
		Log.d("CustomCrop[State]: ", "Finished");
		return new Rect(offsetLeft, offsetTop, width, height);
	}
	public void readQR()
	{
		Rect crop = CustomCrop(1280, 960, 600, 175, 600, 425);
		Bitmap bitmapSrc = Bitmap.createBitmap(crop.width, crop.height, Bitmap.Config.ARGB_8888); 
		String content = null;
		int loopCount = 0; 			
		final int loopCountMax = 5;

		while(!QRCodeFinish && loopCount < loopCountMax)
		{
			Log.d("Move&GetImage[State]:", " Starting");
			long timeStart = SystemClock.elapsedRealtime();
			moveTo(Point_A, Quaternion_Target);
			if(loopCount > 2)	crop = CustomCrop(1280, 960,0,0,0,0);
			Mat matSrc = new Mat(api.getMatNavCam(), crop);
			Log.d("Move&GetImage[State]:", " Finished");
			Log.d("Move&GetImage[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);
			try
			{
				Log.d("QR[State]:", " Starting");
				timeStart = SystemClock.elapsedRealtime();

				matToBitmap(matSrc, bitmapSrc, false);
				int[] pixel = new int[bitmapSrc.getWidth() * bitmapSrc.getHeight()];
				bitmapSrc.getPixels(pixel, 0, bitmapSrc.getWidth(), 0, 0, bitmapSrc.getWidth(), bitmapSrc.getHeight());
				Image QR_code = new Image(bitmapSrc.getWidth(), bitmapSrc.getHeight(), "RGB4");
				QR_code.setData(pixel);
				ImageScanner reader = new ImageScanner();
				reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
				reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);
				reader.scanImage(QR_code.convert("Y800"));
				SymbolSet Symbols = reader.getResults();
				for (Symbol Sym : Symbols)
				{
					content = Sym.getData();
					Log.e("QR[State]:", " Detected");
					Log.d("QR[Data][Raw]:", " " + content);
				}
			}
			catch (Exception error)
			{
				Log.e("QR[State]:", " Not detected");
				Log.e("QR[Error]: ", error.getMessage());
			}
			finally
			{
				if(content != null)
				{
					api.sendDiscoveredQR(content);
					double[] Buf = StringParseToDouble(content);
					Pattern = (int) Buf[0];
					Point_A_Prime = new Point(Buf[1], Buf[2], Buf[3]);
					Log.d("QR[Data][Pattern]:", " " + Pattern);
					Log.d("QR[Data][A_Prime]:", " X: " + Point_A_Prime.getX() + ", Y: " + Point_A_Prime.getY() + ", Z: " + Point_A_Prime.getZ());
					QRCodeFinish = true;
				}
				Log.d("QR[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);
			}
			Log.d("A_Event[State]:", " Finished");
			Log.d("A_Event[Count]:", " " + loopCount);
			loopCount++;
		}
	}
	public void getTargetPosition()
	{
		Mat matSrc = new Mat();
		Mat IDs = new Mat();
		List<Mat> Corners = new ArrayList<>();
		Dictionary Dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
		int[] AR_ID = new int[]{0, 0, 0, 0};
		int[] origin_crop = {520, 800};
		Rect crop = CustomCrop(1280, 960, origin_crop[1], 0, origin_crop[0], 480);
		int loopCount = 0;
		final int loopCountMax = 5;
		int[] 	ArID1_CornerRT = new int[]{0, 0},
				ArID2_CornerLT = new int[]{0, 0},
				ArID3_CornerLB = new int[]{0, 0},
				ArID4_CornerRB = new int[]{0, 0};

		while(!ARCodeFinish && loopCount < loopCountMax)
		{
			Log.d("AR[State]:", " Starting");
			long timeStartAR = SystemClock.elapsedRealtime();

			for(int i=0; i<2; i++)
			{
				moveTo(Point_A, Quaternion_Target);
				matSrc = new Mat(api.getMatNavCam(), crop);
			}
			try
			{
				Aruco.detectMarkers(matSrc, Dict, Corners, IDs);
				AR_ID = new int[]
						{
								(int) IDs.get(0, 0)[0],
								(int) IDs.get(1, 0)[0],
								(int) IDs.get(2, 0)[0],
								(int) IDs.get(3, 0)[0]
						};
				Log.d("AR[State]:", " Detected");
				Log.d("AR[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStartAR) / 1000);
			}
			catch (Exception error)
			{
				Log.e("AR[State]:", " Not detected");
				Log.e("AR[Error]: ", error.getMessage());
			}
			finally
			{
				if (AR_ID[0] != 0 && AR_ID[1] != 0 && AR_ID[2] != 0 && AR_ID[3] != 0)
				{
					Log.d("TargetCal[State]:", " Starting");
					long timeStart2 = SystemClock.elapsedRealtime();

					for (int i = 0; i < 4; i++)
					{
						double[][] Corner =
								{
										{(int) Corners.get(i).get(0, 0)[0], (int) Corners.get(i).get(0, 0)[1]},
										{(int) Corners.get(i).get(0, 2)[0], (int) Corners.get(i).get(0, 2)[1]},
										{(int) Corners.get(i).get(0, 1)[0], (int) Corners.get(i).get(0, 1)[1]},
										{(int) Corners.get(i).get(0, 3)[0], (int) Corners.get(i).get(0, 3)[1]}
								};
						for(int j=0; j<4; j++)
							Log.d("AR_Corner[" + i + "][" + j + "]:", " X= " + Corner[j][0] + ", Y= " + Corner[j][1]);
						if (AR_ID[i] == 1){ ArID1_CornerRT[0] = (int) Corner[2][0]; ArID1_CornerRT[1] = (int) Corner[2][1]; }
						else if (AR_ID[i] == 2){ ArID2_CornerLT[0] = (int) Corner[0][0]; ArID2_CornerLT[1] = (int) Corner[0][1]; }
						else if (AR_ID[i] == 3){ ArID3_CornerLB[0] = (int) Corner[3][0]; ArID3_CornerLB[1] = (int) Corner[3][1]; }
						else if (AR_ID[i] == 4){ ArID4_CornerRB[0] = (int) Corner[1][0]; ArID4_CornerRB[1] = (int) Corner[1][1]; }
					}
					org.opencv.core.Point[] cornerIN =
							{
									new org.opencv.core.Point(ArID2_CornerLT[0] + origin_crop[0], ArID2_CornerLT[1] + origin_crop[1]),
									new org.opencv.core.Point(ArID1_CornerRT[0] + origin_crop[0], ArID1_CornerRT[1] + origin_crop[1]),
									new org.opencv.core.Point(ArID4_CornerRB[0] + origin_crop[0], ArID4_CornerRB[1] + origin_crop[1]),
									new org.opencv.core.Point(ArID3_CornerLB[0] + origin_crop[0], ArID3_CornerLB[1] + origin_crop[1])
							};
					Log.d("ARCorner(ID1)", " X= " + cornerIN[0].x + ", Y= " + cornerIN[0].y);
					Log.d("ARCorner(ID2)", " X= " + cornerIN[1].x + ", Y= " + cornerIN[1].y);
					Log.d("ARCorner(ID3)", " X= " + cornerIN[2].x + ", Y= " + cornerIN[2].y);
					Log.d("ARCorner(ID4)", " X= " + cornerIN[3].x + ", Y= " + cornerIN[3].y);
					findTargetPoint(cornerIN);
					getQuafromTargetPoint();
					Log.d("TargetCal[Data][Point]:", " X: " + Point_Target.getX() + ", Y: " + Point_Target.getY() + ", Z: " + Point_Target.getZ());
					Log.d("TargetCal[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart2));
					ARCodeFinish = true;
				}
				Log.d("AR+Cal[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStartAR) / 1000);
			}
			loopCount++;
		}
	}
	public void getQuafromTargetPoint()
	{
		double [] robot_pos		= {Point_A.getX(), Point_A.getY(), Point_A.getZ()};
		double [] target_pos	= {Point_Target.getX(), Point_Target.getY(), Point_Target.getZ()};
		double [] target_vec	= getVector(target_pos, robot_pos);
		double target_val		= getVal(target_pos);
		double [] laser_shift	= {0.1302, 0.0572, -0.111} ;
		double laser_val		= getVal (laser_shift);
		double las_angle		= 136.11699 * 0.01745;
		double [] length		= getRoot(1,-2*laser_val*Math.cos(las_angle),(laser_val*laser_val)-(target_val*target_val));

		double laser_length;
		if (length[0] > 0 ){laser_length = length[0];}
		else {laser_length = length[1];}

		double [] origin_vec	= {laser_shift[0]+laser_length, laser_shift[1], laser_shift[2]};
		double [] Qua			= getQua(origin_vec, target_vec);
		Quaternion_Target = new Quaternion((float)Qua[1], (float)Qua[2], (float)Qua[3], (float)Qua[0]);
		Log.d("FinalQuaternion2= ", "w: " + Qua[0] + ", a: " + Qua[1] + ", b: " + Qua[2] +", c: " + Qua[3]);
	}
	public void findTargetPoint(org.opencv.core.Point[] image_point)
	{
		org.opencv.core.Point3[] object_point =
				{
						new org.opencv.core.Point3(-0.1375, -0.0665, 0),
						new org.opencv.core.Point3( 0.1375, -0.0665, 0),
						new org.opencv.core.Point3( 0.1375,  0.0665, 0),
						new org.opencv.core.Point3(-0.1375,  0.0665, 0)
				};
		MatOfPoint2f corners1 = new MatOfPoint2f();
		MatOfPoint3f corners2 = new MatOfPoint3f();
		corners1.fromArray(image_point);
		corners2.fromArray(object_point);
		double[][] intrinsics = api.getNavCamIntrinsics();
		double[] ArrayCameraMatrix = intrinsics[0];
		double[] ArrayDistCoeffs = intrinsics[1];
		Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
		cameraMatrix.put(0, 0, ArrayCameraMatrix);
		Log.d("CameraMatrix", cameraMatrix.dump());
		Mat distCoeffs = new Mat(1, 5, CvType.CV_64F);
		distCoeffs.put(0, 0, ArrayDistCoeffs);
		Log.d("DistCoeffs", distCoeffs.dump());
		Mat rvec = new Mat(),
				tvec = new Mat();
		Calib3d.solvePnP(corners2, corners1, cameraMatrix, new MatOfDouble(distCoeffs), rvec, tvec, false, Calib3d.CV_ITERATIVE);
		Log.d("[SolvePnP_rvec]", rvec.dump());
		Log.d("[SolvePnP_tvec]", tvec.dump());
		Mat rotateMatrix = new Mat();
		Calib3d.Rodrigues(rvec, rotateMatrix);
		Log.d("[SolvePnP_rotationMat]", rotateMatrix.dump());
		Mat TransTargetToCam0 = new Mat(4, 4, CvType.CV_64F);
		List<Mat> srcConcat = Arrays.asList(rotateMatrix, tvec);
		Core.hconcat(srcConcat, TransTargetToCam0);
		final double[] ArrayBotMat = {0, 0 , 0, 1};
		Mat botMat = new Mat (1, 4, CvType.CV_64F);
		botMat.put(0, 0, ArrayBotMat);
		Mat TransTargetToCam = new Mat();
		List<Mat> srcConcat2 = Arrays.asList(TransTargetToCam0, botMat);
		Core.vconcat(srcConcat2, TransTargetToCam);
		Log.d("[SolvePnP_TransToImg]", TransTargetToCam.dump());
		double focalLength = -0.204418;
		TransTargetToCam.put(2,3, TransTargetToCam.get(2, 3)[0] + focalLength);
		Log.d("[SolvePnP_TransToCam]",  TransTargetToCam.dump());
		double[] ArrayRotateAxis = {0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1};
		Mat RotateAxis = new Mat(4, 4, CvType.CV_64F);
		RotateAxis.put(0, 0, ArrayRotateAxis);
		Log.d("[SolvePnP_RotateCam]", RotateAxis.dump());
		double  Qx = Quaternion_Target.getX(), Qy = Quaternion_Target.getY(), Qz = Quaternion_Target.getZ(), Qw = Quaternion_Target.getW();
		Mat rotate = new Mat(3, 3, CvType.CV_64F);
		final double[] ArrayRotate =
				{
						Qw*Qw + Qx*Qx - Qy*Qy - Qz*Qz,	2*(Qx*Qy - Qw*Qz),				2*(Qw*Qy + Qx*Qz),
						2*(Qx*Qy + Qw*Qz),				Qw*Qw - Qx*Qx + Qy*Qy - Qz*Qz,	2*(Qy*Qz - Qw*Qx),
						2*(Qx*Qz - Qw*Qy),				2*(Qw*Qx + Qy*Qz),				Qw*Qw - Qx*Qx - Qy*Qy + Qz*Qz
				};
		rotate.put(0, 0, ArrayRotate);
		Mat T = new Mat(3, 1, CvType.CV_64F);
		final double[] ArrayTrans = {0, 0, 0};
		T.put(0, 0, ArrayTrans);
		Mat Rotate0 = new Mat(3, 4, CvType.CV_64F);
		srcConcat = Arrays.asList(rotate, T);
		Core.hconcat(srcConcat, Rotate0);
		Mat RotateCam = new Mat(4, 4, CvType.CV_64F);
		srcConcat2 = Arrays.asList(Rotate0, botMat);
		Core.vconcat(srcConcat2, RotateCam);
		Log.d("[SolvePnP_TransToRobot]", RotateCam.dump());
		double[] ArrayTargetPosePaper = {0, 0, 0, 1};
		Mat TargetPosePaper = new Mat(4, 1, CvType.CV_64F);
		TargetPosePaper.put(0, 0, ArrayTargetPosePaper);
		Log.d("[SolvePnP_PoseOnPaper]", TargetPosePaper.dump());
		Mat PoseOnCam           = new Mat(4, 1, CvType.CV_64F);
		Mat PoseOnCamRotateAxis = new Mat(4, 1, CvType.CV_64F);
		Mat PoseOnCamRobotAxis  = new Mat(4, 1, CvType.CV_64F);
		Mat PoseOnRobot         = new Mat(4, 1, CvType.CV_64F);
		Core.gemm(TransTargetToCam, TargetPosePaper, 1, new Mat(), 0, PoseOnCam);
		Log.d("[SolvePnP_PoseOnCam]", PoseOnCam.dump());
		Core.gemm(RotateAxis, PoseOnCam, 1, new Mat(), 0, PoseOnCamRotateAxis);
		Log.d("[SolvePnP_PoseOnCamRotateAxis]", PoseOnCamRotateAxis.dump());
		Core.gemm(RotateCam, PoseOnCamRotateAxis, 1, new Mat(), 0, PoseOnCamRobotAxis);
		Log.d("[SolvePnP_PoseOnCamRobotAxis]", PoseOnCamRobotAxis.dump());
		Mat RobotToCamNeutral   = new Mat(4, 1, CvType.CV_64F);
		Mat RobotToCam          = new Mat(4, 1, CvType.CV_64F);
		double[] ArrayRobotToCamNeutral = {0.1177, -0.0422, -0.0826, 1};
		RobotToCamNeutral.put(0, 0, ArrayRobotToCamNeutral);
		Core.gemm(RotateCam, RobotToCamNeutral, 1, new Mat(), 0, RobotToCam);
		Log.d("[CamToRobot]", RobotToCam.dump());
		PoseOnRobot.put(0,0, PoseOnCamRobotAxis.get(0,0)[0] + RobotToCam.get(0,0)[0]);
		PoseOnRobot.put(1,0, PoseOnCamRobotAxis.get(1,0)[0] + RobotToCam.get(1,0)[0]);
		PoseOnRobot.put(2,0, PoseOnCamRobotAxis.get(2,0)[0] + RobotToCam.get(2,0)[0]);
		PoseOnRobot.put(3,0, 1);
		Log.d("[SolvePnP_PoseOnRobot]", PoseOnRobot.dump());
		Point_Target = new Point(   Point_A.getX() + PoseOnCamRobotAxis.get(0,0)[0],
				-10.585,
				Point_A.getZ() + PoseOnCamRobotAxis.get(2,0)[0]);
	}
}
