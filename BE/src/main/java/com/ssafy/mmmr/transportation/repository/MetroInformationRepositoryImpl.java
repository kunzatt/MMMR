package com.ssafy.mmmr.transportation.repository;

import java.util.List;
import java.util.Optional;

import com.querydsl.core.BooleanBuilder;
import com.querydsl.jpa.impl.JPAQueryFactory;
import com.ssafy.mmmr.transportation.entity.MetroInformationEntity;
import com.ssafy.mmmr.transportation.entity.QMetroInformationEntity;

import jakarta.persistence.EntityManager;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class MetroInformationRepositoryImpl implements MetroInformationRepositoryCustom {

	private final EntityManager entityManager;
	private final JPAQueryFactory queryFactory;

	@Override
	public List<MetroInformationEntity> searchByKeyword(String keyword) {
		JPAQueryFactory queryFactory = new JPAQueryFactory(entityManager);
		QMetroInformationEntity metro = QMetroInformationEntity.metroInformationEntity;

		BooleanBuilder builder = new BooleanBuilder();

		if (keyword != null && !keyword.isEmpty()) {
			builder.or(metro.lineNumber.containsIgnoreCase(keyword))
				.or(metro.stationName.containsIgnoreCase(keyword));
		}

		return queryFactory
			.selectFrom(metro)
			.where(builder)
			.orderBy(metro.lineNumber.asc(), metro.stationName.asc())
			.fetch();
	}

	@Override
	public Optional<MetroInformationEntity> findByFlexibleLineNumberAndStationName(String lineNumber, String stationName) {
		QMetroInformationEntity metro = QMetroInformationEntity.metroInformationEntity;

		return Optional.ofNullable(
			queryFactory
				.selectFrom(metro)
				.where(
					metro.lineNumber.contains(lineNumber.replaceAll("호선", "")),
					metro.stationName.eq(stationName)
				)
				.fetchFirst()
		);
	}
}
