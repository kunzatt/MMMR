package com.ssafy.mmmr.businformations.repository;

import java.util.List;
import java.util.Optional;

import com.querydsl.core.BooleanBuilder;
import com.querydsl.jpa.impl.JPAQueryFactory;
import com.ssafy.mmmr.businformations.entity.BusInformationEntity;
import com.ssafy.mmmr.businformations.entity.QBusInformationEntity;

import jakarta.persistence.EntityManager;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class BusInformationRepositoryImpl implements BusInformationRepositoryCustom {

	private final EntityManager entityManager;

	@Override
	public List<BusInformationEntity> searchByKeyword(String keyword) {
		JPAQueryFactory queryFactory = new JPAQueryFactory(entityManager);
		QBusInformationEntity bus = QBusInformationEntity.busInformationEntity;

		BooleanBuilder builder = new BooleanBuilder();

		if (keyword != null && !keyword.isEmpty()) {
			builder.or(bus.route.containsIgnoreCase(keyword))
				.or(bus.station.containsIgnoreCase(keyword));
		}

		return queryFactory
			.selectFrom(bus)
			.where(builder)
			.orderBy(bus.route.asc(), bus.sequence.asc())
			.fetch();
	}

	@Override
	public Optional<BusInformationEntity> findByRouteIdAndStationIdAndRoute(Integer routeId, Integer stationId,
		String route) {
		JPAQueryFactory queryFactory = new JPAQueryFactory(entityManager);
		QBusInformationEntity bus = QBusInformationEntity.busInformationEntity;

		BusInformationEntity result = queryFactory
			.selectFrom(bus)
			.where(bus.routeId.eq(routeId)
				.and(bus.stationId.eq(stationId))
				.and(bus.route.eq(route)))
			.fetchFirst();

		return Optional.ofNullable(result);
	}

	@Override
	public Optional<BusInformationEntity> findByRouteIdAndSequenceAndRoute(Integer routeId, Integer sequence,
		String route) {
		JPAQueryFactory queryFactory = new JPAQueryFactory(entityManager);
		QBusInformationEntity bus = QBusInformationEntity.busInformationEntity;

		BusInformationEntity result = queryFactory
			.selectFrom(bus)
			.where(bus.routeId.eq(routeId)
				.and(bus.sequence.eq(sequence))
				.and(bus.route.eq(route)))
			.fetchFirst();

		return Optional.ofNullable(result);
	}
}
