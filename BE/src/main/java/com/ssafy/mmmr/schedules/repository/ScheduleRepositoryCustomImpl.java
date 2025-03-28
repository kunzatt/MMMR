package com.ssafy.mmmr.schedules.repository;

import java.util.List;
import java.util.Optional;

import org.springframework.util.StringUtils;

import com.querydsl.core.types.dsl.BooleanExpression;
import com.querydsl.jpa.impl.JPAQueryFactory;
import com.ssafy.mmmr.schedules.entity.QScheduleEntity;
import com.ssafy.mmmr.schedules.entity.ScheduleEntity;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class ScheduleRepositoryCustomImpl implements ScheduleRepositoryCustom {

	private final JPAQueryFactory queryFactory;
	private final QScheduleEntity schedule = QScheduleEntity.scheduleEntity;

	@Override
	public List<ScheduleEntity> findAllNotDeleted() {
		return queryFactory
			.selectFrom(schedule)
			.where(isNotDeleted())
			.orderBy(schedule.startDate.desc())
			.fetch();
	}

	@Override
	public Optional<ScheduleEntity> findByIdAndNotDeleted(Long id) {
		ScheduleEntity result = queryFactory
			.selectFrom(schedule)
			.where(
				schedule.id.eq(id),
				isNotDeleted()
			)
			.fetchOne();

		return Optional.ofNullable(result);
	}

	@Override
	public List<ScheduleEntity> findByProfileIdAndNotDeleted(Long profileId) {
		return queryFactory
			.selectFrom(schedule)
			.where(
				schedule.profile.id.eq(profileId),
				isNotDeleted()
			)
			.orderBy(schedule.startDate.desc())
			.fetch();
	}

	@Override
	public List<ScheduleEntity> searchByProfileIdAndKeyword(Long profileId, String keyword) {
		return queryFactory
			.selectFrom(schedule)
			.where(
				schedule.profile.id.eq(profileId),
				isNotDeleted(),
				containsKeyword(keyword)
			)
			.orderBy(schedule.startDate.desc())
			.fetch();
	}

	private BooleanExpression isNotDeleted() {
		return schedule.deleted.eq(false);
	}

	private BooleanExpression containsKeyword(String keyword) {
		return StringUtils.hasText(keyword) ? schedule.title.contains(keyword) : null;
	}
}