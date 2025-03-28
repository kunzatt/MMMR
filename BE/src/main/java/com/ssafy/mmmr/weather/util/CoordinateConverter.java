package com.ssafy.mmmr.weather.util;

import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;

@Slf4j
@Component
public class CoordinateConverter {

	// 기상청 좌표계 기준점
	private static final double EARTH_RADIUS = 6371.00877; // 지구 반경(km)
	private static final double GRID_SPACING = 5.0; // 격자 간격(km)
	private static final double STANDARD_PARALLEL_1 = 30.0; // 표준위도 1
	private static final double STANDARD_PARALLEL_2 = 60.0; // 표준위도 2
	private static final double ORIGIN_LONGITUDE = 126.0; // 기준점 경도
	private static final double ORIGIN_LATITUDE = 38.0; // 기준점 위도
	private static final double ORIGIN_X = 43; // 기준점 X좌표
	private static final double ORIGIN_Y = 136; // 기준점 Y좌표

	public int[] convertToXY(double latitude, double longitude) {
		double DEGRAD = Math.PI / 180.0;
		double re = EARTH_RADIUS / GRID_SPACING;
		double slat1 = STANDARD_PARALLEL_1 * DEGRAD;
		double slat2 = STANDARD_PARALLEL_2 * DEGRAD;
		double originLonRad = ORIGIN_LONGITUDE * DEGRAD;
		double originLatRad = ORIGIN_LATITUDE * DEGRAD;

		double sn = Math.tan(Math.PI * 0.25 + slat2 * 0.5) / Math.tan(Math.PI * 0.25 + slat1 * 0.5);
		sn = Math.log(Math.cos(slat1) / Math.cos(slat2)) / Math.log(sn);
		double sf = Math.tan(Math.PI * 0.25 + slat1 * 0.5);
		sf = Math.pow(sf, sn) * Math.cos(slat1) / sn;
		double ro = Math.tan(Math.PI * 0.25 + originLatRad * 0.5);
		ro = re * sf / Math.pow(ro, sn);

		int[] xy = new int[2];
		double ra = Math.tan(Math.PI * 0.25 + latitude * DEGRAD * 0.5);
		ra = re * sf / Math.pow(ra, sn);
		double theta = longitude * DEGRAD - originLonRad;
		if (theta > Math.PI) theta -= 2.0 * Math.PI;
		if (theta < -Math.PI) theta += 2.0 * Math.PI;
		theta *= sn;

		xy[0] = (int) Math.floor(ra * Math.sin(theta) + ORIGIN_X + 0.5);
		xy[1] = (int) Math.floor(ro - ra * Math.cos(theta) + ORIGIN_Y + 0.5);

		return xy;
	}

}
