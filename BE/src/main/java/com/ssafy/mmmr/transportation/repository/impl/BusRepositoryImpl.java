package com.ssafy.mmmr.transportation.repository.impl;

import java.util.List;

import com.querydsl.jpa.impl.JPAQueryFactory;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.transportation.entity.BusEntity;
import com.ssafy.mmmr.transportation.entity.QBusEntity;
import com.ssafy.mmmr.transportation.repository.BusRepositoryCustom;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class BusRepositoryImpl implements BusRepositoryCustom {

	private final JPAQueryFactory queryFactory;

	@Override
	public List<BusEntity> findActiveBusesByProfile(ProfileEntity profile) {
		QBusEntity bus = QBusEntity.busEntity;

		return queryFactory
			.selectFrom(bus)
			.where(bus.profile.eq(profile)
				.and(bus.deleted.eq(false)))
			.fetch();
	}

	@Override
	public int countActiveBusesByProfile(ProfileEntity profile) {
		QBusEntity bus = QBusEntity.busEntity;

		return (int) queryFactory
			.selectFrom(bus)
			.where(bus.profile.eq(profile)
				.and(bus.deleted.eq(false)))
			.fetchCount();
	}
}