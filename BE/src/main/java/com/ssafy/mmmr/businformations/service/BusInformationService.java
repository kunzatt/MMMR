package com.ssafy.mmmr.businformations.service;

import com.monitorjbl.xlsx.StreamingReader;
import com.ssafy.mmmr.businformations.entity.BusInformationEntity;
import com.ssafy.mmmr.businformations.repository.BusInformationRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.BusInformationException;
import com.ssafy.mmmr.global.error.exception.BusinessException;

import lombok.RequiredArgsConstructor;
import org.apache.poi.ss.usermodel.*;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

@Service
@RequiredArgsConstructor
public class BusInformationService {

	private final BusInformationRepository busInformationRepository;

	private static final int BATCH_SIZE = 500;

	@Transactional
	public int importExcelFile(MultipartFile file) throws IOException {
		int totalImported = 0;
		List<BusInformationEntity> batchList = new ArrayList<>(BATCH_SIZE);

		try (InputStream is = file.getInputStream()) {
			// 대용량 처리를 위한 StreamingReader 사용
			Workbook workbook = StreamingReader.builder()
				.rowCacheSize(100)    // 캐시할 행 수
				.bufferSize(4096)     // 버퍼 크기
				.open(is);

			Sheet sheet = workbook.getSheetAt(0);  // 첫 번째 시트 사용

			boolean isFirstRow = true;  // 헤더 행 건너뛰기 용도

			for (Row row : sheet) {
				if (isFirstRow) {
					isFirstRow = false;
					continue;  // 헤더 행 건너뛰기
				}

				try {
					BusInformationEntity busInfo = parseRowToBusInformation(row);
					if (busInfo != null) {
						batchList.add(busInfo);
						totalImported++;

						// 배치 크기에 도달하면 저장 후 리스트 비우기
						if (batchList.size() >= BATCH_SIZE) {
							busInformationRepository.saveAll(batchList);
							batchList.clear();
						}
					}
				} catch (Exception e) {
					// 로그 남기기
					throw new BusInformationException(ErrorCode.EXCEL_PROCESSING_ERROR);
				}
			}

			// 남은 데이터 저장
			if (!batchList.isEmpty()) {
				busInformationRepository.saveAll(batchList);
			}
		} catch (IOException e) {
			throw new BusInformationException(ErrorCode.FILE_UPLOAD_ERROR);
		} catch (BusInformationException e) {
			throw e;
		} catch (Exception e) {
			throw new BusInformationException(ErrorCode.EXCEL_PROCESSING_ERROR);
		}

		return totalImported;
	}

	private BusInformationEntity parseRowToBusInformation(Row row) {
		try {
			// 셀 값 가져오기, 숫자형은 반드시 숫자로 변환
			int routeId = (int) getNumericCellValue(row.getCell(0));
			String route = getStringCellValue(row.getCell(1));
			int sequence = (int) getNumericCellValue(row.getCell(2));
			int stationId = (int) getNumericCellValue(row.getCell(3));
			String station = getStringCellValue(row.getCell(4));

			// 필수 값 검증
			if (route == null || route.isEmpty() || station == null || station.isEmpty()) {
				return null;
			}

			return BusInformationEntity.builder()
				.routeId(routeId)
				.route(route)
				.sequence(sequence)
				.stationId(stationId)
				.station(station)
				.build();

		} catch (BusinessException e) {
			throw e;
		} catch (Exception e) {
			throw new BusInformationException(ErrorCode.INVALID_INPUT_VALUE);
		}
	}

	private double getNumericCellValue(Cell cell) {
		if (cell == null) {
			return 0;
		}

		switch (cell.getCellType()) {
			case NUMERIC:
				return cell.getNumericCellValue();
			case STRING:
				try {
					return Double.parseDouble(cell.getStringCellValue().trim());
				} catch (NumberFormatException e) {
					return 0;
				}
			default:
				return 0;
		}
	}

	private String getStringCellValue(Cell cell) {
		if (cell == null) {
			return "";
		}

		switch (cell.getCellType()) {
			case STRING:
				return cell.getStringCellValue().trim();
			case NUMERIC:
				return String.valueOf((int) cell.getNumericCellValue());
			default:
				return "";
		}
	}
}